// Copyright 2018 Platina Systems, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
package ethernet

import (
	"fmt"
	"net"

	"github.com/platinasystems/vnet"
	"github.com/platinasystems/vnet/internal/dbgvnet"
	"github.com/platinasystems/xeth"
)

// in case we need bridge attributes beyond what's available in PortEntry
type bridgeEntry struct {
	port          *vnet.PortEntry   // bridge contains port which is network interface (port has stag and Net/netns)
	_macToIfindex map[Address]int32 // DA to ifindex of bridge member (FIXME replaced by map in fe1, remove when fdb pushed to linux)
}

// pipe_port can only appear with one ctag (or eventually untagged) per master
type fdbBridgeMember struct {
	stag      uint16
	pipe_port uint16
}

// FIXME ifindex for bridge is not global, must include netns
type fdbBridgeIndex struct {
	bridge int32
	member int32
	port   int32
}

// map TH fdb stag/pipe_port to linux netns/ifindex for bridge and bridge-member
// fe1 reports learning on fdbBrm, convert to fdbBri when reporting to linux
var fdbBrmToBri = map[fdbBridgeMember]fdbBridgeIndex{}

// learned by pipe_port, lookup bridge member via portvid
type PortFdbInfo struct {
	PipePort uint16
	PortVid  uint16
}

var PipePortByPortVid map[uint16]uint16 // FIXME remove, not needed for learning lookup

var bridgeByStag map[uint16]*bridgeEntry

func (br *bridgeEntry) String() (dump string) {
	dump = fmt.Sprintf("%v, si %v, stag %v, addr %v\n",
		br.port.Ifname, vnet.SiByIfindex[br.port.Ifindex], br.port.Stag, br.port.StationAddr)
	dump += fmt.Sprintf("_macToIfindex: %v entries\n", len(br._macToIfindex))
	for addr, ifindex := range br._macToIfindex {
		dump += fmt.Sprintf(" %v learned on ifindex %v", addr.String(), ifindex)
		be := vnet.PortsByIndex[ifindex]
		if be != nil {
			dump += fmt.Sprintf(" %v", be.Ifname)
		}
		dump += fmt.Sprintf("\n")
	}
	return
}

// number of bridge members on a port
func numBrmOnPort(ifindex int32) (brgMems uint8) {
	for _, bri := range fdbBrmToBri {
		if bri.port == ifindex {
			brgMems++
		}
	}
	return
}

// return si for XETH_DEVTYPE_PORT egress to reach da
func (br *bridgeEntry) LookupSiCtag(da Address, v *vnet.Vnet) (si vnet.Si, ctag uint16) {
	var fdbBri fdbBridgeIndex
	var fdbBrm fdbBridgeMember
	var err error

	hw_addr := make(net.HardwareAddr, 6)
	hw_addr = da[:]

	fdbBrm.stag = br.port.Stag
	fdbBrm.pipe_port, err = v.BridgeMemberLookup(br.port.Stag, hw_addr)
	fdbBri = fdbBrmToBri[fdbBrm]
	dbgvnet.Bridge.Logf("br fe1 lookup[%v] %+v=>%+v, err %v", hw_addr, fdbBrm, fdbBri, err)

	if fdbBri.member == 0 {
		si = vnet.SiNil
	} else {
		brm := vnet.PortsByIndex[fdbBri.member]
		si = vnet.SiByIfindex[fdbBri.member]
		ctag = brm.Ctag
		dbgvnet.Bridge.Logf("br stag %v, ctag %v, si %v, type %v",
			brm.Stag, ctag, si, brm.Devtype)
	}
	return
}

// called from fe1 to init map so vnet can call br api by pipe-port
func (m *Main) SetPortMap(fi []PortFdbInfo) {
	if PipePortByPortVid == nil {
		PipePortByPortVid = make(map[uint16]uint16)
	}
	for _, v := range fi {
		// ignore meth ports
		if v.PortVid != 0 {
			PipePortByPortVid[v.PortVid] = v.PipePort
		}
	}
	dbgvnet.Bridge.Logf("%+v", PipePortByPortVid)
}

// add/remove port from bridge, no change to netns
// update fdb map of stag/pipeport to ifindex of bridge member
func ProcessChangeUpper(msg *xeth.MsgChangeUpper, action vnet.ActionType, v *vnet.Vnet) (err error) {
	var fdbBrm fdbBridgeMember

	switch action {
	case vnet.PostReadyVnetd:
	case vnet.Dynamic:
	default:
		err = dbgvnet.Bridge.Logf("error: unexpected action: %v, %+v", action, msg)
		return
	}

	portUpper := vnet.GetPortByIndex(msg.Upper) // port contained by bridge
	if portUpper == nil {
		err = dbgvnet.Bridge.Logf("upper %v, port not found", msg.Upper)
		return
	} else if portUpper.Devtype != xeth.XETH_DEVTYPE_LINUX_BRIDGE {
		return
	}

	brUpper := bridgeByStag[portUpper.Stag]
	if brUpper == nil {
		err = dbgvnet.Bridge.Logf("upper %v, br not found", msg.Upper)
		return
	}

	portLower := vnet.GetPortByIndex(msg.Lower)
	if portLower == nil {
		err = dbgvnet.Bridge.Logf("lower %v, not found", msg.Lower)
		return
	}
	if portLower.Devtype != xeth.XETH_DEVTYPE_LINUX_VLAN &&
		portLower.Devtype != xeth.XETH_DEVTYPE_LINUX_VLAN_BRIDGE_PORT {
		dbgvnet.Bridge.Logf("lower[%v] type=%v not vlan, not supported as brm",
			msg.Lower,
			portLower.Devtype)
		return
	}

	if msg.Linking == 0 {
		if portLower.Stag != 0 {
			fdbBrm.stag = portLower.Stag
			fdbBrm.pipe_port = PipePortByPortVid[portLower.PortVid]
			if fdbBri, ok := fdbBrmToBri[fdbBrm]; ok {
				dbgvnet.Bridge.Logf("brm del %+v, %+v, br.stag:%v, portvid:%v",
					fdbBrm, fdbBri, brUpper.port.Stag, portLower.PortVid)
				delete(fdbBrmToBri, fdbBrm)
				v.BridgeMemberAddDelHook(fdbBrm.stag, vnet.SiByIfindex[fdbBri.member],
					fdbBrm.pipe_port, portLower.Ctag, false, numBrmOnPort(fdbBri.port))
				portLower.Stag = 0
				portLower.Devtype = xeth.XETH_DEVTYPE_LINUX_VLAN
			} else {
				dbgvnet.Bridge.Logf("fdbBri not found %+v", fdbBrm)
			}
		}
	} else {
		fdbBrm.stag = brUpper.port.Stag
		fdbBrm.pipe_port = PipePortByPortVid[portLower.PortVid]

		fdbBri, ok := fdbBrmToBri[fdbBrm]
		if ok {
			dbgvnet.Bridge.Logf("brm add, REPLACE %+v %+v", fdbBrm, fdbBri) // FIXME cleanup fe1 if this ever happens
			delete(fdbBrmToBri, fdbBrm)
		}
		fdbBri.bridge = msg.Upper
		fdbBri.member = msg.Lower
		fdbBri.port = portLower.Iflinkindex

		dbgvnet.Bridge.Logf("brm add %+v, %+v, br.stag=%v, portvid %v",
			fdbBrm, fdbBri, brUpper.port.Stag, portLower.PortVid)
		fdbBrmToBri[fdbBrm] = fdbBri

		// indexed by portvid/stag in map
		portLower.Stag = brUpper.port.Stag

		v.BridgeMemberAddDelHook(fdbBrm.stag, vnet.SiByIfindex[fdbBri.member],
			fdbBrm.pipe_port, portLower.Ctag, true, numBrmOnPort(fdbBri.port))
		portLower.Devtype = xeth.XETH_DEVTYPE_LINUX_VLAN_BRIDGE_PORT
	}
	return
}

func GetBridgeBySi(si vnet.Si) (br *bridgeEntry) {
	for _, b := range bridgeByStag {
		bsi := vnet.SiByIfindex[b.port.Ifindex]
		if bsi == si {
			br = b
			break
		}
	}
	return br
}

// Bridge interface is mapped by name and by stag
// entry is allocated when creating map by ifname
// stag may change, so that map must be refreshed
func SetBridge(stag uint16, ifname string) *vnet.PortEntry {
	dbgvnet.Bridge.Logf("set br %v %v", stag, ifname)
	if bridgeByStag == nil {
		bridgeByStag = make(map[uint16]*bridgeEntry)
	}
	br := bridgeByStag[stag]
	if br == nil {
		br = new(bridgeEntry)
		br._macToIfindex = make(map[Address]int32)
		bridgeByStag[stag] = br
	}
	pe := vnet.SetPort(ifname)
	if pe.Stag != 0 {
		if pe.Stag != stag {
			dbgvnet.Bridge.Logf("br stag changed %v -> %v", pe.Stag, stag) // FIXME update br
		}
	}
	pe.Stag = stag
	br.port = pe
	return pe
}

func UnsetBridge(stag uint16) {
	br := bridgeByStag[stag]
	if br != nil {
		dbgvnet.Bridge.Logf("delete br %v, stag %v/%v", br.port.Ifname, stag, br.port.Stag)
		vnet.UnsetPort(br.port.Ifname)
		br.port = nil
		delete(bridgeByStag, stag)
	}
}

// push fdb updates to linux
func goSviFromFe() {
	var fdbBrm fdbBridgeMember
	var fdbBri fdbBridgeIndex

	for msg := range vnet.SviFromFeCh {
		br := bridgeByStag[msg.Stag]
		if br == nil {
			dbgvnet.Bridge.Logf("br not found %+v", msg)
		} else {
			switch {
			case msg.MsgId == vnet.MSG_SVI_FDB_ADD:
				fdbBrm.stag = msg.Stag
				fdbBrm.pipe_port = msg.PipePort
				fdbBri = fdbBrmToBri[fdbBrm]
				br._macToIfindex[msg.Addr] = fdbBri.member
			case msg.MsgId == vnet.MSG_SVI_FDB_DELETE:
				delete(br._macToIfindex, msg.Addr)
			}
		}
	}
}

// l2-mod-fifo learning, aging, and flush is posted async from fe1 to vnet
func StartFromFeReceivers() {
	if vnet.SviFromFeCh == nil {
		vnet.SviFromFeCh = make(chan vnet.FromFeMsg, 32)
		go goSviFromFe()
	}
}
