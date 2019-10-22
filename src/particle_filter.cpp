/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

/**
 * init Initializes particle filter by initializing particles to Gaussian
 *   distribution around first position and all the weights to 1.
 * @param x Initial x position [m] (simulated estimate from GPS)
 * @param y Initial y position [m]
 * @param theta Initial orientation [rad]
 * @param std[] Array of dimension 3 [standard deviation of x [m],
 *   standard deviation of y [m], standard deviation of yaw [rad]]
 */
void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // Set number of  particles
  num_particles = 1000;

  // Set random engine for generating noise
  std::default_random_engine gen;

  // Creates normal (Gaussian) distributions for x, y, theta, given the noises
  // and positions in input
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Creating a particle to assign data to
  Particle currentParticle;

  for (int i = 0; i < num_particles; ++i) {

    currentParticle.id = i+1;                 // Assigning an id
    currentParticle.x = dist_x(gen);          // Sampling from x distribution
    currentParticle.y = dist_y(gen);          // Sampling from y distribution
    currentParticle.theta = dist_theta(gen);  // Sampling from theta distribution
    currentParticle.weight = 1.0;             // Assigning a weight = 1

    // Append particle to vector
    particles.push_back (currentParticle);
  };

  //  Update the initialization flag
  is_initialized = true;
}

/**
 * prediction Predicts the state for the next time step
 *   using the process model.
 * @param delta_t Time between time step t and t+1 in measurements [s]
 * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
 *   standard deviation of y [m], standard deviation of yaw [rad]]
 * @param velocity Velocity of car from t to t+1 [m/s]
 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
 */
void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    // Set random engine for generating noise
    std::default_random_engine gen;

    // Create normal (Gaussians) distribution for x, y, theta given the noises
    // in input and mean = 0.0
    normal_distribution<double> dist_p_x(0.0, std_pos[0]);
    normal_distribution<double> dist_p_y(0.0, std_pos[1]);
    normal_distribution<double> dist_p_theta(0.0, std_pos[2]);

    // Update particles' position given Bycicle Model

    // Helper variables
    Particle currentParticle;

    double x0 = 0.0;      // Initial x
    double y0 = 0.0;      // Initial y
    double theta0 = 0.0;  // Initial theta
    double xf = 0.0;      // Final x
    double yf = 0.0;      // Final y
    double thetaf = 0.0;  // Final theta

    double const vOverThetaDot = velocity/yaw_rate;

    // Iterate over particles
    for (int i = 0; i < num_particles; ++i) {

      // Extraxt the particle
      currentParticle = particles[i];

      // Update quantities
      x0 = currentParticle.x;
      y0 = currentParticle.y;
      theta0 = currentParticle.theta;

      // BYCICLE MODEL
      xf = x0 + vOverThetaDot * (sin(theta0 + (yaw_rate * delta_t)) -
                  sin(theta0));
      yf = y0 + vOverThetaDot * (cos(theta0) -
                  cos(theta0 + (yaw_rate * delta_t)));
      thetaf = theta0 + (yaw_rate * delta_t);

      // Add noise
      xf += dist_p_x(gen);
      yf += dist_p_y(gen);
      thetaf += dist_p_theta(gen);

      // Update particle with new values
      currentParticle.x = xf;
      currentParticle.y = yf;
      currentParticle.theta = thetaf;

      // Re-insert the particle in the vector
      particles[i] = currentParticle;
    };
}

/**
 * dataAssociation Finds which observations correspond to which landmarks
 *   (likely by using a nearest-neighbors data association).
 * @param predicted Vector of predicted landmark observations
 * @param observations Vector of landmark observations
 */
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
    // Helper variables
    LandmarkObs currentObservation, currentPrediction, closestPrediction;
    double current_dist;

    // Iterate over observed Landmarks
    for (int i = 0; i < observations.size(); ++i) {

      // Extraxt the landmark
      currentObservation = observations[i];

      // Initialize minum distance and particle index
      double min_dist = 99999.99;
      int min_index = 0;

      // Iterate over predicted Landmarks
      for (int j = 0; j < predicted.size(); ++j) {

        // Extract the particle
        currentPrediction = predicted[j];

        // Use distance calculator defined in the helper functions
        current_dist = dist(currentPrediction.x, currentPrediction.y,
                            currentObservation.x, currentObservation.y);

        // Check distances and update
        if (current_dist < min_dist ) {
          min_dist = current_dist;
          min_index = j;
        };
      };

      // Identify closest landmark
      closestPrediction = predicted[min_index];

      // Assign to the observed landmark the id of the closest predicted one
      currentObservation.id = closestPrediction.id;
      observations[i] = currentObservation;
    };
}

/**
 * updateWeights Updates the weights for each particle based on the likelihood
 *   of the observed measurements.
 * @param sensor_range Range [m] of sensor
 * @param std_landmark[] Array of dimension 2
 *   [Landmark measurement uncertainty [x [m], y [m]]]
 * @param observations Vector of landmark observations
 * @param map Map class containing map landmarks
 */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    // Helper variables
    Particle currentParticle;

    double xp = 0.0;      // Particle x
    double yp = 0.0;      // Particle y
    double thetap = 0.0;  // Particle theta

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double coeff3 = 1.0 / (2 * M_PI * sigma_x * sigma_y);

    vector<LandmarkObs> transformed, predicted;

    // Iterate over particles
    for (int i = 0; i < num_particles; ++i) {

      currentParticle = particles[i];

      xp = currentParticle.x;
      yp = currentParticle.y;
      thetap = currentParticle.theta;

      // -----------------------------------------------------------------------
      // STEP 1 - Transform landmark observations from car coordinate frame to
      // map coordinate frame
      LandmarkObs currentObs, transformedObs;

      double xc = 0.0;  // Landmark x in car ref. frame
      double yc = 0.0;  // Landmark y in car ref. frame
      double xm = 0.0;  // Landmark x in map ref. frame
      double ym = 0.0;  // Landmark y in map ref. frame

      for (int j = 0; j < observations.size(); j++) {

        currentObs = observations[j];

        xc = currentObs.x;
        yc = currentObs.y;

        xm = xp + (xc * cos(thetap)) - (yc * sin(thetap));
        ym = yp + (xc * sin(thetap)) + (yc * cos(thetap));

        transformedObs.x = xm;
        transformedObs.y = ym;
        // NOTE the following id will be modified by the next step
        transformedObs.id = currentObs.id;

        transformed.push_back(transformedObs);
      }
      // -----------------------------------------------------------------------
      // STEP 2 - Associate transformed observations (measurements) with
      // predicted landmarks within range
      LandmarkObs currentLandmark;
      double current_dist;

      // First create a vector of landmarks predicted within range from the
      // landmark map.
      // Iterate over landmarks in the map
      for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

        currentLandmark.x = map_landmarks.landmark_list[k].x_f;
        currentLandmark.y = map_landmarks.landmark_list[k].y_f;
        currentLandmark.id = map_landmarks.landmark_list[k].id_i;

        // check if landmark is in range from the particle, given sensor range
        current_dist = dist(xp, yp, currentLandmark.x, currentLandmark.y);

        if (current_dist <= sensor_range){
          predicted.push_back(currentLandmark);
        }
      }

      // Second, use data association function to associate predicted and
      // observed landmark
      dataAssociation(predicted,transformed);

      // After this the vector of trandformed observation has, for each element,
      // the id of the closest landmark from the list in the map

      // -----------------------------------------------------------------------
      // STEP 3 - Calculate the probablities of incurring in the given
      // observations for the given particle.
      // For this we calculate the mutivariate gaussian probability of each
      // observation

      // Init prob, mu
      double cumulatedProb = 1.0;
      double mu_x = 0.0;
      double mu_y = 0.0;

      double coeff1 = 0.0;
      double coeff2 = 0.0;

      double check_dist;

      for (int l = 0; l < transformed.size(); l++) {

        // The x and y means are from the nearest landmark, which id is stored
        // in transformed id
        // NOTE: iterators are 0-based while id start from 1
        mu_x = map_landmarks.landmark_list[transformed[l].id - 1].x_f;
        mu_y = map_landmarks.landmark_list[transformed[l].id - 1].y_f;

        check_dist = dist(transformed[l].x,transformed[l].y,mu_x,mu_y);

        coeff1 = (pow((transformed[l].x - mu_x),2.0) / (2 * pow(sigma_x,2.0)));
        coeff2 = (pow((transformed[l].y - mu_y),2.0) / (2 * pow(sigma_y,2.0)));

        cumulatedProb *= coeff3 * exp (-(coeff1 + coeff2));
      }
      // -----------------------------------------------------------------------

      // Update particle weight and reassign it
      currentParticle.weight = cumulatedProb;
      particles[i] = currentParticle;

      // Clear vectors before next iteration
      predicted.clear();
      transformed.clear();
    }
}

/**
 * resample Resamples from the updated set of particles to form
 *   the new set of particles.
 */
void ParticleFilter::resample() {
   // SAMPLING WHEEL Resample Algorithm

   //
   // p3 = []
   // index = int(random.random()*N)
   // beta = 0.0
   // mw = max(w)
   //
   // for i in range(N):
   //     beta += random.random()*2.0*mw
   //     while beta > w[index]:
   //         beta -= w[index]
   //         index = (index +1) % N
   //     p3.append(p[index])
   //
   // p = p3

   std::cout << "======== RESAMPLING IN ++++++++++++++++++++" << std::endl;
   // Set random engine for generating noise
   std::default_random_engine gen;

   // Determin maximum weight for current particles
   double highest_weight = -1.0;
   for (int i = 0; i < num_particles; ++i) {
     if (particles[i].weight > highest_weight) {
       highest_weight = particles[i].weight;
     }
   }
   std::cout << " Highest weight: " << highest_weight << std::endl;


   // Uniform distributions for index and beta
   uniform_int_distribution<int> dist_index(1, num_particles);
   uniform_real_distribution<int> dist_beta(0, 2 * highest_weight);

   // Initialize new vector and coefficient beta
   std::vector<Particle> resampledParticles;
   double beta = 0.0;

   // Get starting index randomly
   int index = dist_index(gen);

   // Iterate over particles
   for (int j = 0; j < num_particles; ++j) {

     // Increment beta with random generator value
     beta += dist_beta(gen);

     // Check if weight of the particle is greater then beta, and update
     // accordingly
     while (beta > particles[index].weight) {
       beta -= particles[index].weight;
       index = (index + 1) % num_particles;
     }

     // Push sample particle in the new vector
     resampledParticles.push_back(particles[index]);
   }

   // Re-assign the vector of particles
   particles = resampledParticles;
   std::cout << "======== RESAMPLING OUT ++++++++++++++++++++" << std::endl;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
