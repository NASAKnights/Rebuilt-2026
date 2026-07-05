#include "utils/NetworkTableMap.h"

#include <gtest/gtest.h>

#include <array>
#include <string>

TEST(NetworkTableMapTest, SupportsMixedTypedRowsWithNumericKey) {
  NetworkTableMap<double, double, int, bool> table(
      "Test/MixedNumeric",
      "Distance",
      std::array<std::string, 3>{"HoodAngle", "ShotCount", "Enabled"});

  table.Set(2.5, 12.5, 4, true);

  const auto row = table.Get(2.5);
  ASSERT_TRUE(row.has_value());
  EXPECT_EQ(std::get<0>(*row), 12.5);
  EXPECT_EQ(std::get<1>(*row), 4);
  EXPECT_EQ(std::get<2>(*row), true);
}

TEST(NetworkTableMapTest, SupportsStringKeyAndStringRowValues) {
  NetworkTableMap<std::string, std::string, int, bool> table(
      "Test/StringKey",
      "State",
      std::array<std::string, 3>{"Mode", "Priority", "Armed"});

  table.Set("ScoreHigh", std::string("Fast"), 2, false);

  const auto row = table.Get("ScoreHigh");
  ASSERT_TRUE(row.has_value());
  EXPECT_EQ(std::get<0>(*row), "Fast");
  EXPECT_EQ(std::get<1>(*row), 2);
  EXPECT_EQ(std::get<2>(*row), false);
}

TEST(NetworkTableMapTest, SupportsIntegerAndBooleanKeys) {
  NetworkTableMap<int, double, bool> intKeyTable(
      "Test/IntKey",
      "StateId",
      std::array<std::string, 2>{"Value", "Enabled"});
  intKeyTable.Set(7, 3.14, true);

  const auto intRow = intKeyTable.Get(7);
  ASSERT_TRUE(intRow.has_value());
  EXPECT_EQ(std::get<0>(*intRow), 3.14);
  EXPECT_EQ(std::get<1>(*intRow), true);

  NetworkTableMap<bool, int, std::string> boolKeyTable(
      "Test/BoolKey",
      "Active",
      std::array<std::string, 2>{"Count", "Label"});
  boolKeyTable.Set(true, 9, std::string("Enabled"));

  const auto boolRow = boolKeyTable.Get(true);
  ASSERT_TRUE(boolRow.has_value());
  EXPECT_EQ(std::get<0>(*boolRow), 9);
  EXPECT_EQ(std::get<1>(*boolRow), "Enabled");
}
