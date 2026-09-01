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

#include "System/Factory.h"

#include <memory>
#include <string>

#include "System.h"

namespace ORB_SLAM3 {

using std::string;

SystemFactory::Expected SystemFactory::create(
    const std::shared_ptr<Settings> &settings, bool initFr,
    const string &strSequence) {
  if (!settings->validate()) {
    return tl::make_unexpected(ExpectedError::fmt("Settings do not validate"));
  }

  // Cannot use make_shared with friend constructors?
  auto sys = std::shared_ptr<System>(new System(settings, initFr, strSequence));

  // Initialization must occur separately because we use shared_from_this
  if (!sys->initialize()) {
    return tl::make_unexpected(
        ExpectedError::fmt("Unable to initialize SLAM system"));
  }

  return sys;
}

SystemFactory::Expected SystemFactory::create(const std::string &configFile,
                                              const SensorType sensor,
                                              bool initFr,
                                              const string &strSequence) {
  auto exSettings = SettingsLoader::Load(configFile, sensor);

  if (!exSettings) {
    return tl::make_unexpected(ExpectedError::fmt("Unable to load settings"));
  }

  return SystemFactory::create(exSettings.value(), initFr, strSequence);
}

SystemFactory::Expected SystemFactory::create(const std::string &configFile,
                                              const std::string &vocabFile,
                                              const SensorType sensor,
                                              bool initFr,
                                              const string &strSequence) {
  auto exSettings = SettingsLoader::Load(configFile, sensor, vocabFile);

  if (!exSettings) {
    return tl::make_unexpected(ExpectedError::fmt("Unable to load settings"));
  }

  auto settings = exSettings.value();
  return SystemFactory::create(settings, initFr, strSequence);
}

}  // namespace ORB_SLAM3
