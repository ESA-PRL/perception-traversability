#include <boost/test/unit_test.hpp>
#include <traversability/Dummy.hpp>

using namespace traversability;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    traversability::DummyClass dummy;
    dummy.welcome();
}
