// Copyright (c) 2026 Rockwell Automation, Inc. All rights reserved.
// Portions of this file were generated with AI assistance.
// SPDX-License-Identifier: BSD-3-Clause

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "ewellix_driver/ewellix_serial/ewellix_serial.hpp"

using ewellix_driver::EwellixSerial;

TEST(Cycle2Data, ParsesAllActuatorsAndFiveErrorEntries)
{
  std::vector<uint8_t> data(EwellixSerial::CYCLE2_DATA_SIZE, 0);
  data[20] = 123;
  data[78] = 1;
  data[94] = 2;

  EwellixSerial::Cycle2Data state(data);

  EXPECT_EQ(state.actual_positions[5], 123);
  ASSERT_EQ(state.errors.size(), 5u);
  EXPECT_EQ(state.errors.front().code, 1);
  EXPECT_EQ(state.errors.back().code, 2);
}

TEST(Cycle2Data, RejectsInvalidResponseSizes)
{
  EwellixSerial::Cycle2Data state;
  EXPECT_THROW(state.setFromData(std::vector<uint8_t>(97, 0)), std::invalid_argument);
  EXPECT_THROW(state.setFromData(std::vector<uint8_t>(99, 0)), std::invalid_argument);
}
