//
// Created by Anton Matosov on 6/19/17.
//

#define BOOST_TEST_MODULE SerialTests

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(first_test)
{
    int i = -1;
    BOOST_TEST(i > 0);
}
