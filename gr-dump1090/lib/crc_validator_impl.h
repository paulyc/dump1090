/* -*- c++ -*- */
/* 
 * Copyright 2018 $Paul Ciarlo <paul.ciarlo@gmail.com>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_DUMP1090_CRC_VALIDATOR_IMPL_H
#define INCLUDED_DUMP1090_CRC_VALIDATOR_IMPL_H

#include <dump1090/crc_validator.h>

namespace gr {
  namespace dump1090 {

    class crc_validator_impl : public crc_validator
    {
     private:
      // Nothing to declare in this block.

     public:
      crc_validator_impl(double);
      ~crc_validator_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace dump1090
} // namespace gr

#endif /* INCLUDED_DUMP1090_CRC_VALIDATOR_IMPL_H */

