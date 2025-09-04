/*
 * Force.h
 *
 *  Created on: Mar 25, 2021
 *      Author: leandro
 */

#pragma once
#include<vector>
#include "Particle.h"


class Force {
public:
	virtual void apply(real dt, const std::vector<std::unique_ptr<Particle>> &particles) const = 0;
	virtual ~Force() {}
 };
