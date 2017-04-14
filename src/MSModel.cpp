#include "glCanvas.h"

#include <fstream>
#include "MSModel.h"
#include "argparser.h"
#include "utils.h"

// ================================================================================
// ================================================================================

MSModel::MSModel(ArgParser *_args) {
  args =_args;

  // open the file
  std::ifstream istr(std::string(args->path+'/'+args->cloth_file).c_str());
  assert (istr.good());
  std::string token;

  // read in the simulation parameters
  istr >> token >> k_structural; assert (token == "k_structural");  // (units == N/m)  (N = kg*m/s^2)
  istr >> token >> k_shear; assert (token == "k_shear");
  istr >> token >> k_bend; assert (token == "k_bend");
  istr >> token >> damping; assert (token == "damping");
  // NOTE: correction factor == .1, means springs shouldn't stretch more than 10%
  //       correction factor == 100, means don't do any correction
  istr >> token >> provot_structural_correction; assert (token == "provot_structural_correction");
  istr >> token >> provot_shear_correction; assert (token == "provot_shear_correction");

  // the cloth dimensions
  istr >> token >> nx >> ny; 
  assert (token == "m");
  assert (nx >= 2 && ny >= 2);

  // the corners of the cloth
  // (units == meters)
  glm::vec3 a,b,c,d;
  istr >> token >> a.x >> a.y >> a.z; assert (token == "p");
  istr >> token >> b.x >> b.y >> b.z; assert (token == "p");
  istr >> token >> c.x >> c.y >> c.z; assert (token == "p");
  istr >> token >> d.x >> d.y >> d.z; assert (token == "p");

  // fabric weight  (units == kg/m^2)
  // denim ~300 g/m^2
  // silk ~70 g/m^2
  double fabric_weight;
  istr >> token >> fabric_weight; assert (token == "fabric_weight");
  double area = AreaOfTriangle(a,b,c) + AreaOfTriangle(a,c,d);

  // create the particles
  particles = new ClothParticle[nx*ny];
  double mass = area*fabric_weight / double(nx*ny);
  for (int i = 0; i < nx; i++) {
    double x = i/double(nx-1);
    glm::vec3 ab = float(1-x)*a + float(x)*b;
    glm::vec3 dc = float(1-x)*d + float(x)*c;
    for (int j = 0; j < ny; j++) {
      double y = j/double(ny-1);
      ClothParticle &p = getParticle(i,j);
      glm::vec3 abdc = float(1-y)*ab + float(y)*dc;
      p.setOriginalPosition(abdc);
      p.setPosition(abdc);
      p.setVelocity(glm::vec3(0,0,0));
      p.setMass(mass);
      p.setFixed(false);
    }
  }

  // the fixed particles
  while (istr >> token) {
    assert (token == "f");
    int i,j;
    double x,y,z;
    istr >> i >> j >> x >> y >> z;
    ClothParticle &p = getParticle(i,j);
    p.setPosition(glm::vec3(x,y,z));
    p.setFixed(true);
  }

  computeBoundingBox();
}

// ================================================================================

void MSModel::computeBoundingBox() {
	/*
  box = BoundingBox(getParticle(0,0).getPosition());
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      box.Extend(getParticle(i,j).getPosition());
      box.Extend(getParticle(i,j).getOriginalPosition());
    }
  }
  */
}

// ================================================================================



void MSModel::Animate() {


  // *********************************************************************  
  // ASSIGNMENT:
  //
  // Compute the forces on each particle, and update the state
  // (position & velocity) of each particle.
  //
  // Also, this is where you'll put the Provot correction for super-elasticity
  //
  // *********************************************************************    


  // commented out because an animated bounding box can be weird
  //computeBoundingBox();

	glm::vec3 gravity = args->gravity;
	float ts = args->timestep;

	//calculate forces for each particle
	for (int i = 0; i < surfacepoints.size(); i++) {
		SurfacePoint* THEparticle = surfacepoints[i];
		
		if (THEparticle->isFixed()) continue;

		//for soft object simulation, we only need structural springs
		std::vector<SurfacePoint*> neighbors = THEparticle->getNeighbors();

		glm::vec3 force = glm::vec3(0, 0, 0);

		//internal forces on the particle by structural springs
		for (int i = 0; i < neighbors.size(); i++) {
			glm::vec3 spring = THEparticle->getPosition() - neighbors[i]->getPosition();
			float length = glm::length(spring);
			float nlength = glm::distance(THEparticle->getOriginalPosition(), neighbors[i]->getOriginalPosition());

			//force -= (spring - (nlength * glm::normalize(spring))) * (float)k_structural;
			force += ((float)k_structural * (length - nlength) * (spring / length));

			//damping force
			force += (THEparticle->getVelocity - neighbors[i]->getVelocity()) * (float)damping;
		}

		//gravitational force
		force += (float)THEparticle->getMass() * gravity;

		/*
		//explosion prevention - halve timestep
		if (glm::length(force / (float)THEparticle->getMass() * ts) > 10) {
			args->timestep /= 2.0f;
			ts = args->timestep;
			y--;
			continue;
		}
		*/

		//update acceleration, velocity, and position
		THEparticle->setAcceleration(force / (float)THEparticle->getMass());
		THEparticle->setVelocity(THEparticle->getVelocity() + (THEparticle->getAcceleration() * ts));
		THEparticle->setPosition(THEparticle->getPosition() + (THEparticle->getVelocity() * ts));
	}
}

