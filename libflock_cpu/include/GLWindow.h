#ifndef NGLSCENE_H_
#define NGLSCENE_H_


#include "Shader.h"
#include "TrackballCamera.h"
#include "Mesh.h"

#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <ext.hpp>
#include <glm.hpp>
#include <QOpenGLWidget>
#include <QResizeEvent>
#include <QEvent>
#include <memory>
#include "Buffer.h"
#include "Flock.h"
#include <sys/time.h>


class GLWindow : public QOpenGLWidget
{
Q_OBJECT        // must include this if you use Qt signals/slots
public :
  /// @brief Constructor for GLWindow
  //----------------------------------------------------------------------------------------------------------------------
  /// @brief Constructor for GLWindow
  /// @param [in] _parent the parent window to create the GL context in
  //----------------------------------------------------------------------------------------------------------------------
  GLWindow( QWidget *_parent );

    /// @brief dtor
  ~GLWindow();
  void mouseMove( QMouseEvent * _event );
  void mouseClick( QMouseEvent * _event );

  glm::mat4 getProjection(){return m_projection;}

  GLuint getMVPAddress(){return m_MVPAddress;}
  GLuint getMVAddress(){return m_MVAddress;}
  GLuint getNAddress(){return m_NAddress;}

  TrackballCamera getCamera(){return m_camera;}

  int getAmountVertexData(){return m_amountVertexData;}

  void dumpGeo(uint _frameNumber,
               std::vector<Prey> _boids);


public slots:
  void rotating( const bool _rotating ) { m_rotating = _rotating; }
  void init();
    void generateNewGeometry();
    void numBoidsInput(const int _numBoids);
protected:
  /// @brief  The following methods must be implimented in the sub class
  /// this is called when the window is created
  void initializeGL();

  /// @brief this is called whenever the window is re-sized
  /// @param[in] _w the width of the resized window
  /// @param[in] _h the height of the resized window
  void resizeGL(int _w , int _h);
  /// @brief this is the main gl drawing routine which is called whenever the window needs to be re-drawn
  void paintGL();
  void renderScene();

  int m_frame_count = 0;



private :
  //----------------------------------------------------------------------------------------------------------------------
  Mesh * m_mesh;
  //----------------------------------------------------------------------------------------------------------------------
  std::array<Mesh, 2>m_meshes;
  //----------------------------------------------------------------------------------------------------------------------
  Shader m_shader;
  //----------------------------------------------------------------------------------------------------------------------
  GLuint m_vao;
  //----------------------------------------------------------------------------------------------------------------------
  GLuint m_vbo;
  //----------------------------------------------------------------------------------------------------------------------
  GLuint m_nbo;
  //----------------------------------------------------------------------------------------------------------------------
  GLint m_vertexPositionAddress;
  //----------------------------------------------------------------------------------------------------------------------
  GLint m_vertexNormalAddress;
  //----------------------------------------------------------------------------------------------------------------------
  GLint m_MVAddress;
  //----------------------------------------------------------------------------------------------------------------------
  GLint m_MVPAddress;
  //----------------------------------------------------------------------------------------------------------------------
  GLint m_NAddress;
  //----------------------------------------------------------------------------------------------------------------------
  glm::mat4 m_projection;
  //----------------------------------------------------------------------------------------------------------------------
  glm::mat4 m_view;
  //----------------------------------------------------------------------------------------------------------------------
  glm::mat4 m_MV;
  //----------------------------------------------------------------------------------------------------------------------
  glm::mat4 m_MVP;
  //----------------------------------------------------------------------------------------------------------------------
  TrackballCamera m_camera;
  //----------------------------------------------------------------------------------------------------------------------
  bool m_rotating;
  //----------------------------------------------------------------------------------------------------------------------
  Buffer m_buffer;
  //----------------------------------------------------------------------------------------------------------------------
  int m_amountVertexData;
  std::vector <float> m_flockVertices;
  std::vector <float> m_flockNormals;

  std::unique_ptr<Flock> m_flock;
};

#endif
