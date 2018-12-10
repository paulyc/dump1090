/* -*- c++ -*- */

#define DUMP1090_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "dump1090_swig_doc.i"

%{
#include "dump1090/mode_s_decoder.h"
#include "dump1090/mode_ac_decoder.h"
#include "dump1090/iq_converter.h"
#include "dump1090/crc_validator.h"
#include "dump1090/demod_ask.h"
%}


%include "dump1090/mode_s_decoder.h"
GR_SWIG_BLOCK_MAGIC2(dump1090, mode_s_decoder);
%include "dump1090/mode_ac_decoder.h"
GR_SWIG_BLOCK_MAGIC2(dump1090, mode_ac_decoder);
%include "dump1090/iq_converter.h"
GR_SWIG_BLOCK_MAGIC2(dump1090, iq_converter);
%include "dump1090/crc_validator.h"
GR_SWIG_BLOCK_MAGIC2(dump1090, crc_validator);
%include "dump1090/demod_ask.h"
GR_SWIG_BLOCK_MAGIC2(dump1090, demod_ask);
