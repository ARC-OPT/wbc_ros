
#include <gtest/gtest.h>
#include "test_wbc.hpp"

// Declare a test
TEST(TestSuite, test_node_status){

}

TEST(TestSuite, test_wbc_output){

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "wbc_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
