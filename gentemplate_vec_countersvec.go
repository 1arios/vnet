// autogenerated: do not edit!
// generated from gentemplate [gentemplate -d Package=vnet -id countersVec -d VecType=CountersVec -d Type=Counters github.com/platinasystems/elib/vec.tmpl]

// Copyright 2016 Platina Systems, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package vnet

import (
	"github.com/platinasystems/elib"
)

type CountersVec []Counters

func (p *CountersVec) Resize(n uint) {
	old_cap := uint(cap(*p))
	new_len := uint(len(*p)) + n
	if new_len > old_cap {
		new_cap := elib.NextResizeCap(new_len)
		q := make([]Counters, new_len, new_cap)
		copy(q, *p)
		*p = q
	}
	*p = (*p)[:new_len]
}

func (p *CountersVec) validate(new_len uint, zero Counters) *Counters {
	old_cap := uint(cap(*p))
	old_len := uint(len(*p))
	if new_len <= old_cap {
		// Need to reslice to larger length?
		if new_len > old_len {
			*p = (*p)[:new_len]
			for i := old_len; i < new_len; i++ {
				(*p)[i] = zero
			}
		}
		return &(*p)[new_len-1]
	}
	return p.validateSlowPath(zero, old_cap, new_len, old_len)
}

func (p *CountersVec) validateSlowPath(zero Counters, old_cap, new_len, old_len uint) *Counters {
	if new_len > old_cap {
		new_cap := elib.NextResizeCap(new_len)
		q := make([]Counters, new_cap, new_cap)
		copy(q, *p)
		for i := old_len; i < new_cap; i++ {
			q[i] = zero
		}
		*p = q[:new_len]
	}
	if new_len > old_len {
		*p = (*p)[:new_len]
	}
	return &(*p)[new_len-1]
}

func (p *CountersVec) Validate(i uint) *Counters {
	var zero Counters
	return p.validate(i+1, zero)
}

func (p *CountersVec) ValidateInit(i uint, zero Counters) *Counters {
	return p.validate(i+1, zero)
}

func (p *CountersVec) ValidateLen(l uint) (v *Counters) {
	if l > 0 {
		var zero Counters
		v = p.validate(l, zero)
	}
	return
}

func (p *CountersVec) ValidateLenInit(l uint, zero Counters) (v *Counters) {
	if l > 0 {
		v = p.validate(l, zero)
	}
	return
}

func (p *CountersVec) ResetLen() {
	if *p != nil {
		*p = (*p)[:0]
	}
}

func (p CountersVec) Len() uint { return uint(len(p)) }
