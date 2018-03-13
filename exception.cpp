/*
 *  exception.cpp
 *  
 *
 *  Created by Danielle Mersch on 15/06/15.
 *  Copyright 2015 __LMB__. All rights reserved.
 *
 */

#include "exception.h"

Exception::Exception(int error, string info){
	display_error(error, info);
}

void Exception::display_error(int e, string info){
	switch (e){
	case CANNOT_OPEN_FILE:
		error_message = "Cannot open file: " + info;
		break;
	case CANNOT_READ_FILE:
		error_message = "Cannot read file: " + info;
		break;
	case NOT_ENOUGH_BYTES:
	  error_message = "Not enough bytes " + info;
		break;
	case USE:
	  error_message = "Usage: " + info;
		break;
	case OUTPUT_EXISTS:
		error_message = "Output file " + info + " exists.";
		break;
	case BUFFER:
	  error_message = "Buffer problem";
		break;	
	case CANNOT_WRITE_FILE:
	  error_message = "Cannot write output file: " + info;
		break;
	case UNKNOWN_OPTION:
	  error_message = "Unknown option: " + info;
		break;
	case ARGUMENT_MISSING:
		error_message = "Option "+ info + " requires an argument.";
		break;
	case INTERNAL_ERROR:
		error_message = "Internal error: " + info;
		break;
  case PARAMETER_ERROR:
  case DATA_ERROR:
  case OPTION_MISSING:
  case NO_INPUT:
  case CAMERA_ERROR:
      error_message = info;
      break;
	default:
		error_message = "Unknown error";
		break;
	}	
	cerr<<error_message<<endl;
}


string Exception::get_error(){
	return error_message;
}
