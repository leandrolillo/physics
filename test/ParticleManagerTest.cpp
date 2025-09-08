#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"
#include "Gravity.h"

TEST_CASE("ParticleManager") {
  ParticleManager particleManager;
}

TEST_CASE("Forces") {
  Gravity gravity(vector(0, -9.8, 0));
}
