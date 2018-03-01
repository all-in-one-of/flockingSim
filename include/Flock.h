#ifndef Flock_H_
#define Flock_H_
#include <vector>
#include <ngl/Vec3.h>
#include <ngl/Camera.h>
#include "iostream"

#include "Boid.h"


class Flock
{
public :

	/// @brief ctor
    /// @param _pos the position of the Flock
    /// @param _numBoids the number of Boids to create
    Flock(int _numBoids );
    /// @brief a method to update each of the Boids contained in the system
  void update();
    /// @brief a method to draw all the Boids contained in the system
  void draw();
  int getNoBoids(){return m_numBoids;}
  std::vector <Boid> getBoidsVector(){return m_Boids;}

  inline void setCam(ngl::Camera *_cam){m_cam=_cam;}
  inline ngl::Camera * getCam()const {return m_cam;}
  inline void setShaderName(const std::string &_n){m_shaderName=_n;}
  inline const std::string getShaderName()const {return m_shaderName;}


private :
    /// @brief the number of Boids
    int m_numBoids;
    /// @brief the container for the Boids
    std::vector <Boid> m_Boids;
  /// @brief the name of the shader to use
  std::string m_shaderName;
  /// @brief a pointer to the camera used for drawing
  ngl::Camera *m_cam;

};


#endif
