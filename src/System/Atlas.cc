/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <pangolin/pangolin.h>

#include <algorithm>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Converter.h"
#include "System.h"
#include "Utils/Checksum.h"

namespace ORB_SLAM3 {

void System::SaveAtlas(FileType type) {
  const string mStrSaveAtlasToFile = settings_->atlasSaveFile();

  if (!mStrSaveAtlasToFile.empty()) {
    // clock_t start = clock();

    // Save the current session
    mpAtlas->PreSave();

    string pathSaveFileName = "./";
    pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
    pathSaveFileName = pathSaveFileName.append(".osa");

    const string vocabularyFilePath = settings_->strVocFile_;

    string strVocabularyChecksum =
        Checksum::Calculate(vocabularyFilePath, FileType::TEXT_FILE);
    std::size_t found = vocabularyFilePath.find_last_of("/\\");
    string strVocabularyName = vocabularyFilePath.substr(found + 1);

    if (type == FileType::TEXT_FILE) {
      // File text

      oslog::debug("Starting to write the save text file ");
      std::remove(pathSaveFileName.c_str());
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      oslog::debug("End to write the save text file");
    } else if (type == FileType::BINARY_FILE) {
      // File binary

      oslog::debug("Starting to write the save binary file");
      std::remove(pathSaveFileName.c_str());
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      oslog::debug("End to write save binary file");
    }
  }
}

bool System::LoadAtlas(FileType type) {
  string strFileVoc, strVocChecksum;

  const string mStrLoadAtlasFromFile = settings_->atlasLoadFile();
  const string vocabularyFilePath = settings_->strVocFile_;
  bool isRead = false;

  string pathLoadFileName = "./";
  pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
  pathLoadFileName = pathLoadFileName.append(".osa");

  if (type == FileType::TEXT_FILE) {
    // File text
    oslog::debug("Starting to read the save text file ");
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      oslog::error("Cannot find Atlas file {}", pathLoadFileName);
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;

    oslog::debug("Finished loading the saved text file ");
    isRead = true;
  } else if (type == FileType::BINARY_FILE) {
    // File binary
    oslog::debug("Starting to read the save binary file");
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      oslog::error("Cannot find Atlas file {}", pathLoadFileName);
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;

    oslog::debug("Finished loading the saved binary file");
    isRead = true;
  }

  if (!mpAtlas) {
    throw std::runtime_error("mpAtlas not initialized when it should be");
  }

  if (isRead) {
    // Check if the vocabulary is the same
    string strInputVocabularyChecksum =
        Checksum::Calculate(vocabularyFilePath, FileType::TEXT_FILE);

    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      oslog::warn(
          "The vocabulary load isn't the same which the load session was "
          "created.  Loading vocab file {}",
          strFileVoc);
      return false;  // Both are differents
    }

    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();

    return true;
  }
  return false;
}

}  // namespace ORB_SLAM3
