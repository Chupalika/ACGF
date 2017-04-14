#include "glCanvas.h"

#include <fstream>
#include "cloth.h"
#include "argparser.h"
#include "utils.h"

// ================================================================================
// ================================================================================

Cloth::Cloth(ArgParser *_args) {
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

void Cloth::computeBoundingBox() {
  box = BoundingBox(getParticle(0,0).getPosition());
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      box.Extend(getParticle(i,j).getPosition());
      box.Extend(getParticle(i,j).getOriginalPosition());
    }
  }
}

// ================================================================================



void Cloth::Animate() {


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
	for (int x = 0; x < nx; x++) {
		for (int y = 0; y < ny; y++) {
			ClothParticle &THEparticle = getParticle(x, y);

			if (THEparticle.isFixed()) continue;

			//gather the neighbors
			std::vector<ClothParticle> structuralneighbors;
			std::vector<ClothParticle> shearneighbors;
			std::vector<ClothParticle> flexionneighbors;

			//structural springs
			if (x + 1 < nx) structuralneighbors.push_back(getParticle(x + 1, y));
			if (x - 1 >= 0) structuralneighbors.push_back(getParticle(x - 1, y));
			if (y + 1 < ny) structuralneighbors.push_back(getParticle(x, y + 1));
			if (y - 1 >= 0) structuralneighbors.push_back(getParticle(x, y - 1));

			//shear springs
			if (x + 1 < nx && y + 1 < ny) shearneighbors.push_back(getParticle(x + 1, y + 1));
			if (x + 1 < nx && y - 1 >= 0) shearneighbors.push_back(getParticle(x + 1, y - 1));
			if (x - 1 >= 0 && y + 1 < ny) shearneighbors.push_back(getParticle(x - 1, y + 1));
			if (x - 1 >= 0 && y - 1 >= 0) shearneighbors.push_back(getParticle(x - 1, y - 1));

			//flexion springs
			if (x + 2 < nx) flexionneighbors.push_back(getParticle(x + 2, y));
			if (x - 2 >= 0) flexionneighbors.push_back(getParticle(x - 2, y));
			if (y + 2 < ny) flexionneighbors.push_back(getParticle(x, y + 2));
			if (y - 2 >= 0) flexionneighbors.push_back(getParticle(x, y - 2));
			
			glm::vec3 force = glm::vec3(0, 0, 0);

			//internal forces on the particle by structural springs
			for (int i = 0; i < structuralneighbors.size(); i++) {
				glm::vec3 spring = THEparticle.getPosition() - structuralneighbors[i].getPosition();
				float length = glm::length(spring);
				float nlength = glm::distance(THEparticle.getOriginalPosition(), structuralneighbors[i].getOriginalPosition());
				force -= (spring - (nlength * glm::normalize(spring))) * (float)k_structural;
			}

			//internal forces on the particle by shear springs
			for (int i = 0; i < shearneighbors.size(); i++) {
				glm::vec3 spring = THEparticle.getPosition() - shearneighbors[i].getPosition();
				float length = glm::length(spring);
				float nlength = glm::distance(THEparticle.getOriginalPosition(), shearneighbors[i].getOriginalPosition());
				force -= (spring - (nlength * glm::normalize(spring))) * (float)k_shear;
			}

			//internal forces on the particle by flexion springs
			for (int i = 0; i < flexionneighbors.size(); i++) {
				glm::vec3 spring = THEparticle.getPosition() - flexionneighbors[i].getPosition();
				float length = glm::length(spring);
				float nlength = glm::distance(THEparticle.getOriginalPosition(), flexionneighbors[i].getOriginalPosition());
				force -= (spring - (nlength * glm::normalize(spring))) * (float)k_bend;
			}

			//external force
			force += (float)THEparticle.getMass() * gravity;

			//damping force
			force -= THEparticle.getVelocity() * (float)damping;

			//std::cout << force.x << "," << force.y << "," << force.z << '\n';

			//explosion prevention - halve timestep
			if (glm::length(force / (float)THEparticle.getMass() * ts) > 10) {
				args->timestep /= 2.0f;
				ts = args->timestep;
				y--;
				continue;
			}

			//update acceleration, velocity, and position
			THEparticle.setAcceleration(force / (float)THEparticle.getMass());
			THEparticle.setVelocity(THEparticle.getVelocity() + (THEparticle.getAcceleration() * ts));
			THEparticle.setPosition(THEparticle.getPosition() + (THEparticle.getVelocity() * ts));

			//std::cout << "new position: " << THEparticle.getPosition().x << "," << THEparticle.getPosition().y << "," << THEparticle.getPosition().z << "\n";
		}
	}

	//super-elastic springs adjustments
	for (int x = 0; x < nx; x++) {
		for (int y = 0; y < ny; y++) {
			ClothParticle &THEparticle = getParticle(x, y);

			//gather the neighbors
			std::vector<ClothParticle> structuralneighbors;
			std::vector<ClothParticle> shearneighbors;
			std::vector<ClothParticle> flexionneighbors;

			//structural springs
			if (x + 1 < nx) structuralneighbors.push_back(getParticle(x + 1, y));
			if (x - 1 >= 0) structuralneighbors.push_back(getParticle(x - 1, y));
			if (y + 1 < ny) structuralneighbors.push_back(getParticle(x, y + 1));
			if (y - 1 >= 0) structuralneighbors.push_back(getParticle(x, y - 1));

			//shear springs
			if (x + 1 < nx && y + 1 < ny) shearneighbors.push_back(getParticle(x + 1, y + 1));
			if (x + 1 < nx && y - 1 >= 0) shearneighbors.push_back(getParticle(x + 1, y - 1));
			if (x - 1 >= 0 && y + 1 < ny) shearneighbors.push_back(getParticle(x - 1, y + 1));
			if (x - 1 >= 0 && y - 1 >= 0) shearneighbors.push_back(getParticle(x - 1, y - 1));

			//adjust the structural springs
			for (int i = 0; i < structuralneighbors.size(); i++) {
				glm::vec3 spring = THEparticle.getPosition() - structuralneighbors[i].getPosition();
				float length = glm::length(spring);
				float nlength = glm::distance(THEparticle.getOriginalPosition(), structuralneighbors[i].getOriginalPosition());
				float deformation = (length - nlength) / nlength;

				glm::vec3 pos1 = THEparticle.getPosition();
				glm::vec3 pos2 = structuralneighbors[i].getPosition();
				//std::cout << "particle (" << x << "," << y << "): " << pos1.x << "," << pos1.y << "," << pos1.z << "\n";
				//std::cout << "neighbor " << i << ": " << pos2.x << "," << pos2.y << "," << pos2.z << "\n";
				//std::cout << "position: " << THEparticle.getPosition().x << "," << THEparticle.getPosition().y << "," << THEparticle.getPosition().z << "\n";
				//std::cout << "length: " << length << "\n";
				//std::cout << "nlength: " << nlength << "\n";

				//if the deformation is larger than threshold...
				if (deformation > provot_structural_correction) {
					//std::cout << "over threshold: " << deformation << " > " << provot_structural_correction << "\n";
					glm::vec3 correction = (glm::normalize(spring) * nlength) * (deformation - (float)provot_structural_correction);

					//if both particles are loose
					if (!(THEparticle.isFixed()) && !(structuralneighbors[i].isFixed())) {\
						THEparticle.setPosition(THEparticle.getPosition() - (correction / 2.0f));
						structuralneighbors[i].setPosition(structuralneighbors[i].getPosition() + (correction / 2.0f));

						THEparticle.setVelocity(glm::vec3(0, 0, 0));
						structuralneighbors[i].setVelocity(glm::vec3(0, 0, 0));
						//THEparticle.setVelocity(THEparticle.getVelocity() * (1.0f - (correction / length) / 2.0f));
						//structuralneighbors[i].setVelocity(structuralneighbors[i].getVelocity() * (1.0f - (correction / length) / 2.0f));
					}
					//else if this particle is loose
					if (!(THEparticle.isFixed()) && structuralneighbors[i].isFixed()) {
						THEparticle.setPosition(THEparticle.getPosition() - correction);

						THEparticle.setVelocity(glm::vec3(0, 0, 0));
						//THEparticle.setVelocity(THEparticle.getVelocity() * (1.0f - correction / length));
					}
					//else if the neighboring particle is loose
					if (THEparticle.isFixed() && !(structuralneighbors[i].isFixed())) {
						structuralneighbors[i].setPosition(structuralneighbors[i].getPosition() + correction);

						structuralneighbors[i].setVelocity(glm::vec3(0, 0, 0));
						//structuralneighbors[i].setVelocity(structuralneighbors[i].getVelocity() * (1.0f - correction / length));
					}
					//otherwise do nothing
				}
			}

			//adjust the shear springs
			for (int i = 0; i < shearneighbors.size(); i++) {
				glm::vec3 spring = THEparticle.getPosition() - shearneighbors[i].getPosition();
				float length = glm::length(spring);
				float nlength = glm::distance(THEparticle.getOriginalPosition(), shearneighbors[i].getOriginalPosition());
				float deformation = (length - nlength) / nlength;

				//if the deformation is larger than threshold...
				if (deformation > provot_shear_correction) {
					glm::vec3 correction = (glm::normalize(spring) * nlength) * (deformation - (float)provot_shear_correction);

					//if both particles are loose
					if (!(THEparticle.isFixed()) && !(shearneighbors[i].isFixed())) {
						THEparticle.setPosition(THEparticle.getPosition() - (correction / 2.0f));
						shearneighbors[i].setPosition(shearneighbors[i].getPosition() + correction);

						THEparticle.setVelocity(glm::vec3(0, 0, 0));
						shearneighbors[i].setVelocity(glm::vec3(0, 0, 0));
						//THEparticle.setVelocity(THEparticle.getVelocity() * (1.0f - (correction / length) / 2.0f));
						//shearneighbors[i].setVelocity(shearneighbors[i].getVelocity() * (1.0f - (correction / length) / 2.0f));
					}
					//else if this particle is loose
					else if (!(THEparticle.isFixed()) && shearneighbors[i].isFixed()) {
						THEparticle.setPosition(THEparticle.getPosition() - correction);

						THEparticle.setVelocity(glm::vec3(0, 0, 0));
						//THEparticle.setVelocity(THEparticle.getVelocity() * (1.0f - correction / length));
					}
					//else if the neighboring particle is loose
					else if (THEparticle.isFixed() && !(shearneighbors[i].isFixed())) {
						shearneighbors[i].setPosition(shearneighbors[i].getPosition() + correction);

						shearneighbors[i].setVelocity(glm::vec3(0, 0, 0));
						//shearneighbors[i].setVelocity(shearneighbors[i].getVelocity() * (1.0f - correction / length));
					}
					//otherwise do nothing
				}
			}
		}
	}
}

