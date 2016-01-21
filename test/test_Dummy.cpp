#include <boost/test/unit_test.hpp>
#include <fps_per_velocity/Dummy.hpp>

using namespace fps_per_velocity;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    fps_per_velocity::DummyClass dummy;
    dummy.welcome();
}
