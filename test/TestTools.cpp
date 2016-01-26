/*
 * TestTools.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: tiagotrocoli
 */

#include "TestTools.hpp"

#include <pocolog_cpp/LogFile.hpp>
#include <pocolog_cpp/Index.hpp>
#include <base/Time.hpp>

pocolog_cpp::InputDataStream* TestTools::getDataStreamFromLog(
    std::string path, std::string stream_name) {

  pocolog_cpp::InputDataStream *data_stream = NULL;
  try {
    pocolog_cpp::LogFile log_file(path);
    pocolog_cpp::Stream *stream = &(log_file.getStream(stream_name));
    data_stream = dynamic_cast<pocolog_cpp::InputDataStream *>(stream);

  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  return data_stream;

}
