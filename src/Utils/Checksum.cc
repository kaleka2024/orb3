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

#include "Utils/Checksum.h"

#include <openssl/md5.h>

#include <cstdio>
#include <fstream>
#include <ios>
#include <string>

#include "Logging.h"
#include "Types.h"

namespace ORB_SLAM3 {

namespace Checksum {

using std::string;

string Calculate(string filename, FileType type) {
  string checksum = "";

  unsigned char c[MD5_DIGEST_LENGTH];

  std::ios_base::openmode flags = std::ios::in;
  if (type == FileType::BINARY_FILE)  // Binary file
    flags = std::ios::in | std::ios::binary;

  std::ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    oslog::error("[E] Unable to open the in file {} for Md5 hash.", filename);
    return checksum;
  }

  MD5_CTX md5Context;
  char buffer[1024];

  MD5_Init(&md5Context);
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    MD5_Update(&md5Context, buffer, count);
  }

  f.close();

  MD5_Final(c, &md5Context);

  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    char aux[10];
    snprintf(aux, sizeof(aux), "%02x", c[i]);
    checksum = checksum + aux;
  }

  return checksum;
}

}  // namespace Checksum

}  // namespace ORB_SLAM3
