#ifndef _CLOTH_H_
#define _CLOTH_H_

#include "argparser.h"
#include "boundingbox.h"
#include "vbo_structs.h"
#include <vector>

// =====================================================================================
// Cloth Particles
// =====================================================================================

class SurfacePoint {
public:
  // ACCESSORS
  const glm::vec3& getOriginalPosition() const{ return original_position; }
  const glm::vec3& getPosition() const{ return position; }
  const glm::vec3& getVelocity() const{ return velocity; }
  const glm::vec3& getAcceleration() const { return acceleration; }
  glm::vec3 getForce() const { return float(mass)*acceleration; }
  double getMass() const { return mass; }
  bool isFixed() const { return fixed; }
  const std::vector<SurfacePoint*> getNeighbors() { return std::vector<SurfacePoint*>; }
  // MODIFIERS
  void setOriginalPosition(const glm::vec3 &p) { original_position = p; }
  void setPosition(const glm::vec3 &p) { position = p; }
  void setVelocity(const glm::vec3 &v) { velocity = v; }
  void setAcceleration(const glm::vec3 &a) { acceleration = a; }
  void setMass(double m) { mass = m; }
  void setFixed(bool b) { fixed = b; 
private:
  // REPRESENTATION
  glm::vec3 original_position;
  glm::vec3 position;
  glm::vec3 velocity;
  glm::vec3 acceleration;
  double mass;
  bool fixed;
};

// =====================================================================================
// Mass-Spring Model
// =====================================================================================

class MSModel {

public:
	MSModel(ArgParser *args);
  ~MSModel() { delete [] surfacepoints; cleanupVBOs(); }

  // ACCESSORS
  const BoundingBox& getBoundingBox() const { return box; }

  // PAINTING & ANIMATING
  void Paint() const;
  void Animate();

  void initializeVBOs();
  void setupVBOs();
  void drawVBOs();
  void cleanupVBOs();

private:

  // PRIVATE ACCESSORS
	//returns a vector of the surfae points that is neighboring this particle
	const std::vector<SurfacePoint*> getNeighbors(SurfacePoint* s) { return 0; }

  glm::vec3 computeGouraudNormal(int i, int j) const;

  // HELPER FUNCTION
  void computeBoundingBox();

  // HELPER FUNCTIONS FOR ANIMATION
  void AddWireFrameTriangle(const glm::vec3 &apos, const glm::vec3 &bpos, const glm::vec3 &cpos,
                            const glm::vec3 &anormal, const glm::vec3 &bnormal, const glm::vec3 &cnormal,
                            const glm::vec3 &abcolor, const glm::vec3 &bccolor, const glm::vec3 &cacolor);

  // REPRESENTATION
  ArgParser *args;
  std::vector<SurfacePoint*> surfacepoints;
  BoundingBox box;
  // simulation parameters
  double damping;
  // spring constants
  double k_structural;

  // VBOs
  GLuint cloth_verts_VBO;
  GLuint cloth_tri_indices_VBO;
  GLuint cloth_velocity_verts_VBO;
  GLuint cloth_velocity_tri_indices_VBO;
  GLuint cloth_force_verts_VBO;
  GLuint cloth_force_tri_indices_VBO;

  // data for rendering
  std::vector<VBOPosNormalColor> cloth_verts; 
  std::vector<VBOIndexedTri> cloth_tri_indices;
  std::vector<VBOPosNormalColor> cloth_velocity_verts; 
  std::vector<VBOIndexedTri> cloth_velocity_tri_indices;
  std::vector<VBOPosNormalColor> cloth_force_verts; 
  std::vector<VBOIndexedTri> cloth_force_tri_indices;
};

// ========================================================================

#endif
