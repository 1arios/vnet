// Copyright 2016 Platina Systems, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package sfp

import (
	"github.com/platinasystems/elib/hw"

	"fmt"
	"strconv"
	"strings"
	"time"
	"unsafe"
)

const (
	qsfp_addr	= 0x50
	sfpp_addr	= 0x51
)

type QsfpThreshold struct {
	Alarm, Warning struct{ Hi, Lo float64 }
}

type QsfpModuleConfig struct {
	TemperatureInCelsius QsfpThreshold
	SupplyVoltageInVolts QsfpThreshold
	RxPowerInWatts       QsfpThreshold
	TxBiasCurrentInAmps  QsfpThreshold
	TxPowerInWatts       QsfpThreshold
}

type QsfpMonitoring struct {
	Temperature string
	Voltage     string
	RxPower     [QsfpNChannel]string
	TxPower     [QsfpNChannel]string
	TxBias      [QsfpNChannel]string
}

type QsfpAlarms struct {
	Module   string
	Channels string
}

type QsfpIdFields struct {
	Id            string
	Vendor        string
	PartNumber    string
	Revision      string
	SerialNumber  string
	Date          string
	ConnectorType string
	Compliance    string
}

type Access interface {
	// Activate/deactivate module reset.
	SfpReset(active bool)
	// Enable/disable low power mode.
	SfpSetLowPowerMode(enable bool)
	SfpReadWrite(addr uint8, offset uint, p []uint8, isWrite bool) (ok bool)
}

type state struct {
	txDisable uint8
}

type QsfpModule struct {
	// Read in when module is inserted and taken out of reset.
	e Eeprom
	// Only compliance values from EEPROM are valid.
	eepromDataplaneSubsetValid bool
	// All values from EEPROM are valid.
	AllEepromValid bool

	signalValues [QsfpNSignal]bool

	Config QsfpModuleConfig

	Ident QsfpIdFields

	Mon QsfpMonitoring

	Alarms QsfpAlarms

	s state
	a Access
	// sfp+ module has a different address for its dynamic monitoring data
	i2c_addr	uint8
}

func getQsfpRegs() *qsfpRegs                   { return (*qsfpRegs)(hw.BasePointer) }
func getQsfpThresholdRegs() *qsfpThresholdRegs { return (*qsfpThresholdRegs)(hw.BasePointer) }
func getEepromRegs(is_sfpp bool) *Eeprom {
	if !is_sfpp {
		r := getQsfpRegs()
		return (*Eeprom)(unsafe.Pointer(hw.BaseAddress + uintptr(r.upperMemory[0].offset())))
	}
	return (*Eeprom)(hw.BasePointer)
}
func getSfppDiagRegs() *sfppDiagRegs		{ return (*sfppDiagRegs)(hw.BasePointer) }

func (r *reg8) offset() uint  { return uint(uintptr(unsafe.Pointer(r)) - hw.BaseAddress) }
func (r *reg16) offset() uint { return uint(uintptr(unsafe.Pointer(r)) - hw.BaseAddress) }

func (r *reg8) get(m *QsfpModule) reg8 {
	var b [1]uint8
	m.a.SfpReadWrite(m.i2c_addr, r.offset(), b[:], false)
	return reg8(b[0])
}

func (r *reg8) set(m *QsfpModule, v uint8) bool {
	var b [1]uint8
	b[0] = v
	return m.a.SfpReadWrite(m.i2c_addr, r.offset(), b[:], true)
}

func (r *reg16) get(m *QsfpModule) (v uint16) {
	var b [2]uint8
	m.a.SfpReadWrite(m.i2c_addr, r.offset(), b[:], false)
	return uint16(b[0])<<8 | uint16(b[1])
}

func (r *reg16) set(m *QsfpModule, v uint16) (ok bool) {
	var b [2]uint8
	b[0] = uint8(v >> 8)
	b[1] = uint8(v)
	return m.a.SfpReadWrite(m.i2c_addr, r.offset(), b[:], true)
}

func (r *regi16) get(m *QsfpModule) (v int16) { v = int16((*reg16)(r).get(m)); return }
func (r *regi16) set(m *QsfpModule, v int16)  { (*reg16)(r).set(m, uint16(v)) }

func (t *QsfpThreshold) get(m *QsfpModule, r *qsfpThreshold, unit float64, castInt16 bool) {
	if castInt16 {
		t.Warning.Hi = float64(int16(r.warning.hi.get(m))) * unit
		t.Warning.Lo = float64(int16(r.warning.lo.get(m))) * unit
		t.Alarm.Hi = float64(int16(r.alarm.hi.get(m))) * unit
		t.Alarm.Lo = float64(int16(r.alarm.lo.get(m))) * unit
	} else {
		t.Warning.Hi = float64(r.warning.hi.get(m)) * unit
		t.Warning.Lo = float64(r.warning.lo.get(m)) * unit
		t.Alarm.Hi = float64(r.alarm.hi.get(m)) * unit
		t.Alarm.Lo = float64(r.alarm.lo.get(m)) * unit
	}
}


const (
	TemperatureToCelsius = 1 / 256.
	SupplyVoltageToVolts = 100e-6
	RxPowerToWatts       = 1e-4
	TxPowerToWatts       = 1e-4
	TxBiasCurrentToAmps  = 2e-3
)

func (m *QsfpModule) SetSignal(s QsfpSignal, new bool) (old bool, err error) {
	old = m.signalValues[s]
	m.signalValues[s] = new
	if old != new {
		switch s {
		case QsfpModuleIsPresent:
			err = m.Present(new)
		case QsfpInterruptStatus:
			if new {
				err = m.Interrupt()
			}
		}
	}
	return
}
func (m *QsfpModule) GetSignal(s QsfpSignal) bool { return m.signalValues[s] }

func (m *QsfpModule) Init(a Access, addr uint8) {
	m.a = a
	m.i2c_addr = addr
}

func (m *QsfpModule) Interrupt() (err error) {
	return
}

func (m *QsfpModule) Present(is bool) (err error) {
	r := getQsfpRegs()
	m.i2c_addr = qsfp_addr
	if !is {
		m.invalidateCache()
	} else {
		m.e.Id = Id((*reg8)(&r.id).get(m))
		if strings.Contains(fmt.Sprintf("%v", m.e.Id), "QSFP") {
			// Wait for module to become ready.
			start := time.Now()
			status := r.status.get(m)
			for status&(1<<0) != 0 || (status == 0) {
				if time.Since(start) >= 100*time.Millisecond {
					err = fmt.Errorf("qsfp status ready timeout")
					return
				}
				status = r.status.get(m)
			}
			// Read enough of EEPROM to keep dataplane happy.
			// Reading eeprom is slow over i2c.
			if r.upperMemoryMapPageSelect.get(m) != 0 {
				r.upperMemoryMapPageSelect.set(m, 0)
			}
			m.eepromDataplaneSubsetValid = false
			m.validateCache(false)
			c, x := m.GetCompliance()
			if x != ExtendedComplianceUnspecified {
				m.Ident.Compliance = fmt.Sprintf("%v", c) + fmt.Sprintf(" %v", x)
			} else {
				m.Ident.Compliance = fmt.Sprintf("%v", c)
			}
		} else if strings.Contains(fmt.Sprintf("%v", m.e.Id), "SFP") {
			r := getSfppDiagRegs()
			m.i2c_addr = sfpp_addr

			start := time.Now()
			status := r.status_ctrl.get(m)
			for status&(1<<0) != 0 {
				if time.Since(start) >= 100*time.Millisecond {
					err = fmt.Errorf("sfp+ status ready timeout")
					return
				}
				status = r.status_ctrl.get(m)
			}
			m.eepromDataplaneSubsetValid = false
			m.validateCache(false)
			c, _ := m.GetCompliance()
			m.Ident.Compliance = fmt.Sprintf("%v", c)
		}
	}
	return
}

func (m *QsfpModule) Monitoring(is_sfpp bool) {
	if !is_sfpp {
		r := getQsfpRegs()
		m.i2c_addr = qsfp_addr

		if r.upperMemoryMapPageSelect.get(m) != 0 {
			r.upperMemoryMapPageSelect.set(m, 0)
		}
		m.Mon.Temperature = strconv.FormatFloat(float64(r.internallyMeasured.temperature.get(m))*TemperatureToCelsius, 'f', 3, 64)
		m.Mon.Voltage = strconv.FormatFloat(float64(r.internallyMeasured.supplyVoltage.get(m))*SupplyVoltageToVolts, 'f', 3, 64)
		for i := 0; i < QsfpNChannel; i++ {
			m.Mon.RxPower[i] = strconv.FormatFloat(float64(r.internallyMeasured.rxPower[i].get(m))*RxPowerToWatts, 'f', 3, 64)
			m.Mon.TxPower[i] = strconv.FormatFloat(float64(r.internallyMeasured.txPower[i].get(m))*TxPowerToWatts, 'f', 3, 64)
			m.Mon.TxBias[i] = strconv.FormatFloat(float64(r.internallyMeasured.txBiasCurrent[i].get(m))*TxBiasCurrentToAmps, 'f', 3, 64)
		}
		r0 := (*reg16)(&r.channelStatusInterrupt).get(m)
		r1 := (*reg16)(&r.monitorInterruptStatus.channelRxPower).get(m)
		r2 := (*reg16)(&r.monitorInterruptStatus.channelTxBiasCurrent).get(m)
		r3 := (*reg16)(&r.monitorInterruptStatus.channelTxPower).get(m)
		r4 := (*reg8)(&r.channelStatusLOL).get(m)
		r5 := (*reg16)(&r.monitorInterruptStatus.module).get(m)
		var channelAlarms string
		var moduleAlarms string
		for i := 0; i < 16; i++ {
			if (1 << uint(i) & r0) != 0 {
				channelAlarms += fmt.Sprintf("%v,", ChannelStatusInterrupt(1<<uint(i)))
			}
			if (1 << uint(i) & r1) != 0 {
				channelAlarms += fmt.Sprintf("%v,", ChannelRxPowerInterrupts(1<<uint(i)))
			}
			if (1 << uint(i) & r2) != 0 {
				channelAlarms += fmt.Sprintf("%v,", ChannelTxBiasInterrupts(1<<uint(i)))
			}
			if (1 << uint(i) & r3) != 0 {
				channelAlarms += fmt.Sprintf("%v,", ChannelTxPowerInterrupts(1<<uint(i)))
			}
			if i < 8 {
				if (1 << uint(i) & r4) != 0 {
					channelAlarms += fmt.Sprintf("%v,", ChannelStatusLOL(1<<uint(i)))
				}
			}
			if (1 << uint(i) & r5) != 0 {
				moduleAlarms += fmt.Sprintf("%v,", ModuleInterrupts(1<<uint(i)))
			}
		}
		if strings.HasSuffix(moduleAlarms, ",") {
			moduleAlarms = moduleAlarms[:len(moduleAlarms)-1]
		}
		if strings.HasSuffix(channelAlarms, ",") {
			channelAlarms = channelAlarms[:len(channelAlarms)-1]
		}

		m.Alarms.Module = moduleAlarms
		m.Alarms.Channels = channelAlarms
	// sfp+
	// uses the same data structures 
	} else {
		r := getSfppDiagRegs()
		m.i2c_addr = sfpp_addr

		m.Mon.Temperature = strconv.FormatFloat(float64(r.internallyMeasured.temperature.get(m))*TemperatureToCelsius, 'f', 3, 64)
		m.Mon.Voltage = strconv.FormatFloat(float64(r.internallyMeasured.supplyVoltage.get(m))*SupplyVoltageToVolts, 'f', 3, 64)
		m.Mon.RxPower[0] = strconv.FormatFloat(float64(r.internallyMeasured.rxPower.get(m))*RxPowerToWatts, 'f', 3, 64)
		m.Mon.TxPower[0] = strconv.FormatFloat(float64(r.internallyMeasured.txPower.get(m))*TxPowerToWatts, 'f', 3, 64)
		m.Mon.TxBias[0] = strconv.FormatFloat(float64(r.internallyMeasured.txBiasCurrent.get(m))*TxBiasCurrentToAmps, 'f', 3, 64)

		r0 := (*reg16)(&r.monitorInterruptStatus.moduleAlarms).get(m)
		r1 := (*reg16)(&r.monitorInterruptStatus.moduleWarnings).get(m)
		var moduleAlarms string
		for i := 0; i < 16; i++ {
			if (1 << uint(i) & r0) != 0 {
				moduleAlarms += fmt.Sprintf("%v,", ModuleAlarms_sfpp(1<<uint(i)))
			}
			if (1 << uint(i) & r1) != 0 {
				moduleAlarms += fmt.Sprintf("%v,", ModuleWarnings_sfpp(1<<uint(i)))
			}
		}
		if strings.HasSuffix(moduleAlarms, ",") {
			moduleAlarms = moduleAlarms[:len(moduleAlarms)-1]
		}

		m.Alarms.Module = moduleAlarms
	}

}

func (m *QsfpModule) validateCache(everything bool) {
	r := getQsfpRegs()
	m.i2c_addr = qsfp_addr

	if everything && !m.AllEepromValid {
		if r.upperMemoryMapPageSelect.get(m) != 0 {
			r.upperMemoryMapPageSelect.set(m, 0)
		}
		// Read whole EEPROM.
		m.AllEepromValid = true

		m.e.Id = Id((*reg8)(&r.id).get(m))
		if strings.Contains(fmt.Sprintf("%v", m.e.Id), "QSFP") {
			p := (*[128]uint8)(unsafe.Pointer(&m.e))
			m.a.SfpReadWrite(qsfp_addr, r.upperMemory[0].offset(), p[:], false)
			// if qsfp is optic read static monitoring thresholds
			if !strings.Contains(m.Ident.Compliance, "CR") && m.Ident.Compliance != "" {
				t := getQsfpThresholdRegs()
				m.i2c_addr = qsfp_addr

				if r.upperMemoryMapPageSelect.get(m) != 3 {
					r.upperMemoryMapPageSelect.set(m, 3)
				}
				m.Config.TemperatureInCelsius.get(m, &t.temperature, TemperatureToCelsius, true)
				m.Config.SupplyVoltageInVolts.get(m, &t.supplyVoltage, SupplyVoltageToVolts, false)
				m.Config.RxPowerInWatts.get(m, &t.rxPower, RxPowerToWatts, false)
				m.Config.TxBiasCurrentInAmps.get(m, &t.txBiasCurrent, TxBiasCurrentToAmps, false)
				m.Config.TxPowerInWatts.get(m, &t.txPower, TxPowerToWatts, false)
			}
		} else if strings.Contains(fmt.Sprintf("%v", m.e.Id), "SFP") {
			p := (*[128]uint8)(unsafe.Pointer(&m.e))
			// eeprom addr is the same for qsfp and sfp+
			m.a.SfpReadWrite(qsfp_addr, 0, p[:], false)
			// if sfp is optic read static monitoring thresholds
			if !strings.Contains(m.Ident.Compliance, "CR") && m.Ident.Compliance != "" {
				t := getSfppDiagRegs()
				m.i2c_addr = sfpp_addr

				m.Config.TemperatureInCelsius.get(m, &t.temperature, TemperatureToCelsius, true)
				m.Config.SupplyVoltageInVolts.get(m, &t.supplyVoltage, SupplyVoltageToVolts, false)
				m.Config.TxBiasCurrentInAmps.get(m, &t.txBiasCurrent, TxBiasCurrentToAmps, false)
				m.Config.TxPowerInWatts.get(m, &t.txPower, TxPowerToWatts, false)
				m.Config.RxPowerInWatts.get(m, &t.rxPower, RxPowerToWatts, false)
			}
		}
	} else if !m.eepromDataplaneSubsetValid {
		// For performance only read fields needed for data plane.

		//if not a qsfp only read Id
		m.eepromDataplaneSubsetValid = true
		m.e.Id = Id((*reg8)(&r.id).get(m))
		if strings.Contains(fmt.Sprintf("%v", m.e.Id), "QSFP") {
			if r.upperMemoryMapPageSelect.get(m) != 0 {
				r.upperMemoryMapPageSelect.set(m, 0)
			}
			er := getEepromRegs(false)
			m.e.ConnectorType = ConnectorType((*reg8)(&er.ConnectorType).get(m))
			m.e.Compatibility[0] = er.Compatibility[0].get(m)
			if Compliance(m.e.Compatibility[0])&ComplianceExtendedValid != 0 {
				m.e.Options[0] = er.Options[0].get(m)
			}
		} else if strings.Contains(fmt.Sprintf("%v", m.e.Id), "SFP") {
			er := getEepromRegs(true)
			m.e.ConnectorType = ConnectorType((*reg8)(&er.ConnectorType).get(m))
			m.e.Compatibility[0] = er.Compatibility[0].get(m)
		}
	}
}

func (m *QsfpModule) invalidateCache() {
	m.AllEepromValid = false
	m.eepromDataplaneSubsetValid = false
}

func (m *QsfpModule) TxEnable(enableMask, laneMask uint) uint {
	r := getQsfpRegs()
	was := m.s.txDisable
	disableMask := byte(^enableMask)
	is := 0xf & ((was &^ byte(laneMask)) | disableMask)
	if is != was {
		r.txDisable.set(m, is)
		m.s.txDisable = is
	}
	return uint(was)
}

func (m *QsfpModule) GetId() Id                       { return m.e.Id }
func (m *QsfpModule) GetConnectorType() ConnectorType { return m.e.ConnectorType }
func (m *QsfpModule) GetCompliance() (c Compliance, x ExtendedCompliance) {
	c = Compliance(m.e.Compatibility[0])
	x = ExtendedComplianceUnspecified
	if c&ComplianceExtendedValid != 0 {
		x = ExtendedCompliance(m.e.Options[0])
	}
	return
}

func trim(r []reg8) string {
	// Strip trailing nulls.
	var b []byte
	for i := 0; i < len(r); i++ {
		if r[i] == 0 {
			break
		}
		b = append(b, byte(r[i]))
	}
	return strings.TrimSpace(string(b))
}

func (m *QsfpModule) String() string {
	m.validateCache(true)
	e := &m.e
	var s string

	m.Ident.Id = fmt.Sprintf("%s", e.Id)
	s = fmt.Sprintf("Id: %s, Compliance: %s, Vendor: %s, Part Number %s, Revision 0x%x, Serial %s, Date %s, Connector Type: %v",
		e.Id, m.Ident.Compliance, trim(e.VendorName[:]), trim(e.VendorPartNumber[:]), trim(e.VendorRevision[:]),
		trim(e.VendorSerialNumber[:]), trim(e.VendorDateCode[:]), e.ConnectorType)

	m.Ident.Vendor = fmt.Sprintf("%s", trim(e.VendorName[:]))
	m.Ident.PartNumber = fmt.Sprintf("%s", trim(e.VendorPartNumber[:]))
	m.Ident.Revision = fmt.Sprintf("0%x", trim(e.VendorRevision[:]))
	m.Ident.SerialNumber = fmt.Sprintf("%s", trim(e.VendorSerialNumber[:]))
	m.Ident.Date = fmt.Sprintf("%s", trim(e.VendorDateCode[:]))
	m.Ident.ConnectorType = fmt.Sprintf("%v", e.ConnectorType)

	return s
}
