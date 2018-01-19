/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <unordered_set>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = 100;
  double init_weight = 1.0/(1.0*num_particles);

  particles.resize(num_particles);
  weights.resize(num_particles);

  default_random_engine gen;

  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std[0]);

  // TODO: Create normal distributions for y and theta
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);



//  cout << "initialize " << num_particles << endl;

  for (int i = 0; i < num_particles; ++i) {
    double sample_x, sample_y, sample_theta;

    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);

    // Print your samples to the terminal.
    particles[i] = Particle{i, sample_x, sample_y, sample_theta, init_weight};
    weights[i] = init_weight;
  }


  is_initialized = true;

//  cout << "init" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;
  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(0, std_pos[0]);

  // TODO: Create normal distributions for y and theta
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);


  for (auto&& p : particles) {

//    cout << p.x << ", " << p.y << endl;
    if (yaw_rate == 0) {
      p.x += velocity*cos(p.theta)*delta_t;
      p.y += velocity*sin(p.theta)*delta_t;
    } else {
      p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta += yaw_rate * delta_t;
    }

    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);
    
//    cout << p.x << ", " << p.y << endl;
//    cout << "---------" << endl;

  }

//  cout << "prediction: "<< "dt=" << delta_t << ", velocity=" << velocity << ", yaw_rate=" << yaw_rate << endl;

}

double ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations,
                                       double std_landmark[]) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

//  std::unordered_set<int> taken = std::unordered_set<int>(observations.size());

  double prob = 1.0;

  for (auto&& o : observations) {
    double distance = std::numeric_limits<double>::max();

    LandmarkObs match;

    for (auto p : predicted) {
//      auto search = taken.find(p.id);
//      if (search == taken.end()) {
        double cd = dist(p.x, p.y, o.x, o.y);
        if (cd < distance) {
          distance = cd;
          o.id = p.id;
          match = p;
        }
//      }
    }

    double p = gaussian_prob(o.x, o.y, match.x, match.y, std_landmark[0], std_landmark[1]);
//    cout << match.id << ", " << o.id << ", " << p << ", " << distance << endl << endl;
    prob *= p;
//    cout << "observation, "<< o.x << ", " << o.y << endl;
//    cout << "predicted, " << match->x << ", " << match->y << endl;
//    cout << "p = " << prob << endl;

//    taken.insert(o.id);
  }

  return prob;

  cout << "data association" << endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  for (auto&& p : particles) {

    // transform observations into map coordinates
    vector<LandmarkObs> map_observations;
    for (auto&& o : observations){
      double mx = p.x + cos(p.theta)*o.x - sin(p.theta)*o.y;
      double my = p.y + sin(p.theta)*o.x + cos(p.theta)*o.y;
      map_observations.push_back(LandmarkObs{o.id, mx, my});
    }

    // find landmarks within sensor_range
    vector<LandmarkObs> predicted;
    for (auto&& l : map_landmarks.landmark_list) {
      if (dist(l.x_f, l.y_f, p.x, p.y) < sensor_range) {
        predicted.push_back(LandmarkObs{l.id_i, l.x_f, l.y_f});
      }
    }

    p.weight = dataAssociation(predicted, map_observations, std_landmark);
    weights[p.id] = p.weight;

//    cout << p.id << ", " << p.weight << ", " << map_observations.size() << ", " << predicted.size() << endl;
//    cout << p.x << ", " << p.y << ", " << p.theta << endl;


    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    for (auto&& o : map_observations) {
      associations.push_back(o.id);
      sense_x.push_back(o.x);
      sense_y.push_back(o.y);
    }
//    SetAssociations(p, associations, sense_x, sense_y);
    p.associations= associations;
    p.sense_x = sense_x;
    p.sense_y = sense_y;
  }

//  cout << "====================" << endl;


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;

  vector<Particle> resampled_particles(num_particles);
  for (int i=0; i < num_particles; ++i) {
    discrete_distribution<int> d(weights.begin(), weights.end());
    Particle* p = &(particles[d(gen)]);
    resampled_particles[i] = Particle{i, p->x, p->y, p->theta, p->weight};
    weights[i] = p->weight;
  }

  particles = resampled_particles;

//  cout << "resample" << endl;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
