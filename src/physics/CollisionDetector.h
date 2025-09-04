/*
 * CollisionDetector.h
 *
 *  Created on: Apr 5, 2021
 *      Author: leandro
 */

#pragma once
constexpr real sphere_radius=0.1;

#include<vector>
#include"ParticleContact.h"
#include"Particle.h"

#include <Geometry.h>
#include "CollisionTester.h"


class CollisionDetector {
    std::vector <std::unique_ptr<Geometry>>scenery;
    CollisionTester intersectionTester;
public:
  CollisionDetector() {
  }
  virtual ~CollisionDetector() {
  }

  const CollisionTester &getIntersectionTester() const {
    return this->intersectionTester;
  }

  Geometry &addScenery(std::unique_ptr<Geometry> scenery) {
      this->scenery.push_back(std::move(scenery));
      return *this->scenery.back();
  }

  const std::vector<std::unique_ptr<Geometry>> &getScenery() const {
    return this->scenery;
  }

  virtual std::vector<ParticleContact>detectCollisions(const std::vector<std::unique_ptr<Particle>> &particles) const {
        std::vector<ParticleContact> contacts;

        for(auto iteratorA = particles.begin(); iteratorA != particles.end(); iteratorA++) {
          auto &particleA = *iteratorA;
          if(particleA && particleA->getStatus()) {
            for(auto &sceneryIterator : this->scenery)  {
              if(sceneryIterator) {
                std::vector<GeometryContact> pairContacts = intersectionTester.detectCollision(particleA->getBoundingVolume(), *sceneryIterator);
                if(!pairContacts.empty()) {
                  std::transform(pairContacts.begin(), pairContacts.end(), std::back_inserter(contacts),
                          [&particleA](GeometryContact pairContact) -> ParticleContact {
                          return ParticleContact(particleA.get(),
                                  null,
                                  pairContact.getIntersection(),
                                  pairContact.getNormal(),
                                  pairContact.getRestitution(),
                                  pairContact.getPenetration());
                  });

                  particleA->onCollision(contacts.back());
                }
              }
            }

            for(auto iteratorB = iteratorA+1; iteratorB != particles.end(); iteratorB++) {
              auto &particleB = *iteratorB;
              if(particleB && particleB->getStatus()) {
                std::vector<GeometryContact> pairContacts = intersectionTester.detectCollision(particleA->getBoundingVolume(), particleB->getBoundingVolume());
                if(!pairContacts.empty()) {
                  std::transform(pairContacts.begin(), pairContacts.end(), std::back_inserter(contacts),
                          [&particleA, &particleB](GeometryContact pairContact) -> ParticleContact {
                          return ParticleContact(particleA.get(),
                                  particleB.get(),
                                  pairContact.getIntersection(),
                                  pairContact.getNormal(),
                                  pairContact.getRestitution(),
                                  pairContact.getPenetration());
                  });

                  particleA->onCollision(contacts.back());
                  particleB->onCollision(contacts.back());
                }
              }
            }
          }
        }

        return contacts;

    }
};
