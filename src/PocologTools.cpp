/*
 * PocologTools.cpp
 *
 *  Created on: Feb 18, 2016
 *      Author: tiagotrocoli
 */

#include "PocologTools.hpp"

#include <pocolog_cpp/LogFile.hpp>
#include <pocolog_cpp/Index.hpp>
#include <base/Time.hpp>

namespace fps_per_velocity {

pocolog_cpp::InputDataStream* PocologTools::getDataStreamFromLog(
    std::string path, std::string stream_name) {

  pocolog_cpp::InputDataStream *data_stream = NULL;
  try {
    pocolog_cpp::LogFile log_file(path);
    data_stream = dynamic_cast<pocolog_cpp::InputDataStream *>(&(log_file
        .getStream(stream_name)));

  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  return data_stream;

}

} /* namespace fps_per_velocity */
