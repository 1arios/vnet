// autogenerated: do not edit!
// generated from gentemplate [gentemplate -d Package=ethernet -id inject_packet_disposition -d VecType=inject_packet_disposition_vec -d Type=inject_packet_disposition github.com/platinasystems/go/elib/vec.tmpl]

// Copyright 2016 Platina Systems, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package ethernet

import (
	"github.com/platinasystems/go/elib"
)

type inject_packet_disposition_vec []inject_packet_disposition

func (p *inject_packet_disposition_vec) Resize(n uint) {
	c := uint(cap(*p))
	l := uint(len(*p)) + n
	if l > c {
		c = elib.NextResizeCap(l)
		q := make([]inject_packet_disposition, l, c)
		copy(q, *p)
		*p = q
	}
	*p = (*p)[:l]
}

func (p *inject_packet_disposition_vec) validate(new_len uint, zero inject_packet_disposition) *inject_packet_disposition {
	c := uint(cap(*p))
	lʹ := uint(len(*p))
	l := new_len
	if l <= c {
		// Need to reslice to larger length?
		if l > lʹ {
			*p = (*p)[:l]
			for i := lʹ; i < l; i++ {
				(*p)[i] = zero
			}
		}
		return &(*p)[l-1]
	}
	return p.validateSlowPath(zero, c, l, lʹ)
}

func (p *inject_packet_disposition_vec) validateSlowPath(zero inject_packet_disposition, c, l, lʹ uint) *inject_packet_disposition {
	if l > c {
		cNext := elib.NextResizeCap(l)
		q := make([]inject_packet_disposition, cNext, cNext)
		copy(q, *p)
		for i := c; i < cNext; i++ {
			q[i] = zero
		}
		*p = q[:l]
	}
	if l > lʹ {
		*p = (*p)[:l]
	}
	return &(*p)[l-1]
}

func (p *inject_packet_disposition_vec) Validate(i uint) *inject_packet_disposition {
	var zero inject_packet_disposition
	return p.validate(i+1, zero)
}

func (p *inject_packet_disposition_vec) ValidateInit(i uint, zero inject_packet_disposition) *inject_packet_disposition {
	return p.validate(i+1, zero)
}

func (p *inject_packet_disposition_vec) ValidateLen(l uint) (v *inject_packet_disposition) {
	if l > 0 {
		var zero inject_packet_disposition
		v = p.validate(l, zero)
	}
	return
}

func (p *inject_packet_disposition_vec) ValidateLenInit(l uint, zero inject_packet_disposition) (v *inject_packet_disposition) {
	if l > 0 {
		v = p.validate(l, zero)
	}
	return
}

func (p *inject_packet_disposition_vec) ResetLen() {
	if *p != nil {
		*p = (*p)[:0]
	}
}

func (p inject_packet_disposition_vec) Len() uint { return uint(len(p)) }
