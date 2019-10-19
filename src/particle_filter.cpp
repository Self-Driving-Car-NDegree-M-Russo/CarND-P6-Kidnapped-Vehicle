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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m],
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   */

  // Set umber of  particles
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

    currentParticle.id = i+1;             // Assigning an id
    currentParticle.x = dist_x(gen);      // Sampling from x distribution
    currentParticle.y = dist_y(gen);      // Sampling from y distribution
    currentParticle.theta = dist_y(gen);  // Sampling from y distribution
    currentParticle.weight = 1.0;         // Sampling from y distribution

    // Append particle to vector
    particles.push_back (currentParticle);

    // Print your samples to the terminal.
    //std::cout << "Sample " << i + 1 << " " << dist_x(gen) << " " << dist_y(gen) << " "
    //          << dist_theta(gen) << std::endl;
  };

  // Print size of the vector
  // std::cout << "Vector size " << particles.size() << std::endl;

  // Print examples, to double check
  // Particle temp1 = particles[234];
  // std::cout << "Temp 1 - id: " << temp1.id << ", x: " << temp1.x << ", y: " << temp1.y
  //         << ", theta: " << temp1.theta << ", w: " << temp1.weight << std::endl;
  //
  // Particle temp2 = particles[538];
  // std::cout << "Temp 2 - id: " << temp2.id << ", x: " << temp2.x << ", y: " << temp2.y
  //         << ", theta: " << temp2.theta << ", w: " << temp2.weight << std::endl;
  //
  // Particle temp3 = particles[183];
  // std::cout << "Temp 3 - id: " << temp3.id << ", x: " << temp3.x << ", y: " << temp3.y
  //         << ", theta: " << temp3.theta << ", w: " << temp3.weight << std::endl;


  //  Update the initialization flag
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */

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

    double x0 = 0.0;
    double y0 = 0.0;
    double theta0 = 0.0;
    double xf = 0.0;
    double yf = 0.0;
    double thetaf = 0.0;

    double const vOverThetaDot = velocity/yaw_rate;

    // Iterate over particles
    for (int i = 0; i < num_particles; ++i) {

      // Extraxt the particle
      currentParticle = particles[i];

      // Update quantities
      x0 = currentParticle.x;
      y0 = currentParticle.y;
      theta0 = currentParticle.theta;

       // std::cout << "CP - id: " << currentParticle.id << ", x: " << x0 << ", y: " << y0
       //          << ", theta: " << theta0 << ", w: " << currentParticle.weight << std::endl;
       // std::cout << "velocity: " << velocity << ", yaw rate: " << yaw_rate << ", delta_t: " << delta_t << std::endl;

      // BYCICLE MODEL
      xf = x0 + vOverThetaDot * (sin(theta0 + (yaw_rate * delta_t)) -
                  sin(theta0));
      yf = y0 + vOverThetaDot * (cos(theta0) -
                  cos(theta0 + (yaw_rate * delta_t)));
      thetaf = theta0 + (yaw_rate * delta_t);

       // std::cout << "xf: " << xf << ", yf: " << yf << ", thetaf: " << thetaf << std::endl;

      // Add noise
      xf += dist_p_x(gen);
      yf += dist_p_y(gen);
      thetaf += dist_p_theta(gen);

       // std::cout << "xf + noise: " << xf << ", yf: " << yf << ", thetaf: " << thetaf << std::endl;

      // Update particle with new values
      currentParticle.x = xf;
      currentParticle.y = yf;
      currentParticle.theta = thetaf;

      // Re-insert the particle in the vector
      particles[i] = currentParticle;
    };
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
   /**
    * dataAssociation Finds which observations correspond to which landmarks
    *   (likely by using a nearest-neighbors data association).
    * @param predicted Vector of predicted landmark observations
    * @param observations Vector of landmark observations
    */

    // Helper variables
    Particle currentParticle, closerParticle;
    LandmarkObs currentLandmark, predictedLandmark;

    double current_dist;

    // Iterate over Landmarks
    for (int i = 0; i < observations.size(); ++i) {

      // Extraxt the landmark
      currentLandmark = observations[i];

      // Initialize minum distance and particle index
      double min_dist = 99999.99;
      int min_index = 0;

      // Iterate over particles
      for (int j = 0; j < num_particles; ++j) {

        // Extract the particle
        currentParticle = particles[j];

        // Use distance calculator defined in the helper functions
        current_dist = dist(currentParticle.x, currentParticle.y,
                            currentLandmark.x, currentLandmark.y);

        // Check distances and uodate
        if (current_dist < min_dist ) {
          min_dist = current_dist;
          min_index = j;
        };
      };

      // Create a landmark from closer particle and send it to vector of
      // predictions
      closerParticle = particles[min_index];

      predictedLandmark.x = closerParticle.x;
      predictedLandmark.y = closerParticle.y;
      predictedLandmark.id = closerParticle.id;

      predicted.push_back (predictedLandmark);
    };
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */



}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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
