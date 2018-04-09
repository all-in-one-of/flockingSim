#include "GLWindow.h"

#include <iostream>
#include <QColorDialog>
#include <QGLWidget>
#include <QImage>
#include <QScreen>
//----------------------------------------------------------------------------------------------------------------------

GLWindow::GLWindow( QWidget *_parent ) : QOpenGLWidget( _parent )
{
  // set this widget to have the initial keyboard focus
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  this->resize( _parent->size() );

  m_flock.reset(new Flock(this, 20));

  m_camera.setInitialMousePos(0,0);
  m_camera.setTarget(0.0f, 0.0f, 0.0f);
  m_camera.setEye(0.0f, 5.0f, 5.0f);
  m_rotating = false;

}

//----------------------------------------------------------------------------------------------------------------------

void GLWindow::initializeGL()
{
//#ifdef linux
  // this needs to be after the context creation, otherwise it GLEW will crash
  //std::cout <<"linux \n";
  glewExperimental = GL_TRUE;
  glewInit();
  //	GLenum error = glGetError();
//#endif
  glEnable( GL_DEPTH_TEST );
  glEnable( GL_MULTISAMPLE );
  glEnable( GL_TEXTURE_2D );
  glClearColor( 0.5f, 0.5f, 0.5f, 1.0f );
  glViewport( 0, 0, devicePixelRatio(), devicePixelRatio() );

  // mesh for boids, can have differen for prey and predator
  m_meshes[0] = Mesh( "../libflock_cpu/models/cone.obj", "cone" );
  //m_meshes[1] = Mesh( "models/cube.obj", "cube" );

  m_mesh = & m_meshes[0];

  m_amountVertexData = m_meshes[0].getAmountVertexData() * m_flock->getNoBoids();

  for(int i = 0; i< m_flock->getNoBoids(); i++)
  {

      // for each boids add a copy of its vertices to total vertices
      m_flockVertices.insert(m_flockVertices.end(), m_meshes[0].getVertices().begin(),m_meshes[0].getVertices().end());

      m_flockNormals.insert(m_flockNormals.end(), m_meshes[0].getNormals().begin(),m_meshes[0].getNormals().end());


  }


  init();
  //m_MV = glm::translate( m_MV, glm::vec3(0.0f, 0.0f, -2.0f) );
}

//----------------------------------------------------------------------------------------------------------------------

void GLWindow::resizeGL( int _w, int _h )
{

}

//----------------------------------------------------------------------------------------------------------------------

GLWindow::~GLWindow()
{

}

//----------------------------------------------------------------------------------------------------------------------

void GLWindow::mouseMove(QMouseEvent * _event)
{
  m_camera.handleMouseMove( _event->pos().x(), _event->pos().y() );

  update();
}

//----------------------------------------------------------------------------------------------------------------------

void GLWindow::mouseClick(QMouseEvent * _event)
{
  m_camera.handleMouseClick(_event->pos().x(), _event->pos().y(), _event->type(), _event, 0);

  update();
}

//----------------------------------------------------------------------------------------------------------------------

void GLWindow::init()
{
  std::string shadersAddress = "../libflock_cpu/shaders/";
  m_shader = Shader( "m_shader", shadersAddress + "phong_vert.glsl", shadersAddress + "simplefrag.glsl" );


  glLinkProgram( m_shader.getShaderProgram() );
  glUseProgram( m_shader.getShaderProgram() );

  glGenVertexArrays( 1, &m_vao );
  glBindVertexArray( m_vao );
  glGenBuffers( 1, &m_vbo );
  glGenBuffers( 1, &m_nbo );

  m_mesh->setBufferIndex( 0 );


  std::vector<float> test = m_mesh[0].getVertices();
  std::vector<float> test2 = m_mesh[1].getVertices();

  std::vector<float> testN = m_mesh[0].getNormals();
  std::vector<float> testN2 = m_mesh[1].getNormals();

  std::cout<<test.size()<<"\n";
  std::cout<<test2.size()<<"\n";

  test.insert(test.end(),test2.begin(),test2.end());

  testN.insert(testN.end(),testN2.begin(),testN2.end());

  std::cout<<test.size()<<"\n";

  m_amountVertexData = m_mesh[0].getAmountVertexData();



  // load vertices
  glBindBuffer( GL_ARRAY_BUFFER, m_vbo );
  glBufferData( GL_ARRAY_BUFFER, m_amountVertexData * sizeof(float), 0, GL_STATIC_DRAW );
  glBufferSubData( GL_ARRAY_BUFFER, 0, m_amountVertexData * sizeof(float), &m_flockVertices[0] );

  // pass vertices to shader
  GLint pos = glGetAttribLocation( m_shader.getShaderProgram(), "VertexPosition" );
  glEnableVertexAttribArray( pos );
  glVertexAttribPointer( pos, 3, GL_FLOAT, GL_FALSE, 0, 0 );

  // load normals
  glBindBuffer( GL_ARRAY_BUFFER,	m_nbo );
  glBufferData( GL_ARRAY_BUFFER, m_amountVertexData * sizeof(float), 0, GL_STATIC_DRAW );
  glBufferSubData( GL_ARRAY_BUFFER, 0, m_amountVertexData * sizeof(float), &m_flockNormals[0] );

  // pass normals to shader
  GLint n = glGetAttribLocation( m_shader.getShaderProgram(), "VertexNormal" );
  glEnableVertexAttribArray( n );
  glVertexAttribPointer( n, 3, GL_FLOAT, GL_FALSE, 0, 0 );

  // link matrices with shader locations
  m_MVAddress = glGetUniformLocation( m_shader.getShaderProgram(), "MV" );
  m_MVPAddress = glGetUniformLocation( m_shader.getShaderProgram(), "MVP" );
  m_NAddress = glGetUniformLocation( m_shader.getShaderProgram(), "N" );
}

//------------------------------------------------------------------------------------------------------------------------------

void GLWindow::paintGL()
{
  glViewport( 0, 0, width(), height() );
  glClearColor( 1, 1, 1, 1.0f );
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  m_flock->update();

  renderScene();

  update();
}

//------------------------------------------------------------------------------------------------------------------------------

void GLWindow::renderScene()
{
  glViewport( 0, 0, width()*devicePixelRatio(), height()*devicePixelRatio() ); //fix for retina screens
  glClearColor( 1, 1, 1, 1.0f );
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  m_camera.update();
  m_projection = glm::perspective( glm::radians( 60.0f ),
                                   static_cast<float>( width() ) / static_cast<float>( height() ), 0.1f, 100.0f );
  m_view = glm::lookAt( glm::vec3( 0.0f, 0.0f, 5.0f ), glm::vec3( 0.0f, 0.0f, 0.0f ), glm::vec3( 0.0f, 1.0f, 0.0f ) );

  m_flock->draw();


  // move
  //m_MV = glm::translate( m_MV, glm::vec3(0.02f, 0.0f, 0.0f) );

//  if ( m_rotating )
//    m_MV = glm::rotate( m_MV, glm::radians( -1.0f ), glm::vec3( 0.0f, 1.0f, 0.0f ) );
//  m_MVP = m_projection * m_camera.viewMatrix() * m_MV;
//  glm::mat3 N = glm::mat3( glm::inverse( glm::transpose( m_MV ) ) );

//  glUniformMatrix4fv( m_MVPAddress, 1, GL_FALSE, glm::value_ptr( m_MVP ) );
//  glUniformMatrix4fv( m_MVAddress, 1, GL_FALSE, glm::value_ptr( m_MV ) );

//  glUniformMatrix3fv( m_NAddress, 1, GL_FALSE, glm::value_ptr( N ) );


//  glDrawArrays( GL_TRIANGLES, 0 , ( m_amountVertexData / 3 ) );
}

//------------------------------------------------------------------------------------------------------------------------------

void GLWindow::generateNewGeometry()
{
  static int count = 0;
  ++count;

  if ( count == m_meshes.size() )
    count = 0;
  m_mesh = &m_meshes[ count ];

  m_amountVertexData = m_mesh->getAmountVertexData();

  m_mesh->setBufferIndex( 0 );

  // load vertices
  glBindBuffer( GL_ARRAY_BUFFER, m_vbo );
  glBufferData( GL_ARRAY_BUFFER, m_amountVertexData * sizeof(float), 0, GL_STATIC_DRAW );
  glBufferSubData( GL_ARRAY_BUFFER, 0, m_mesh->getAmountVertexData() * sizeof(float), &m_mesh->getVertexData() );

  // pass vertices to shader
  GLint pos = glGetAttribLocation( m_shader.getShaderProgram(), "VertexPosition" );
  glEnableVertexAttribArray( pos );
  glVertexAttribPointer( pos, 3, GL_FLOAT, GL_FALSE, 0, 0 );

  // load normals
  glBindBuffer( GL_ARRAY_BUFFER,	m_nbo );
  glBufferData( GL_ARRAY_BUFFER, m_amountVertexData * sizeof(float), 0, GL_STATIC_DRAW );
  glBufferSubData( GL_ARRAY_BUFFER, 0, m_mesh->getAmountVertexData() * sizeof(float), &m_mesh->getNormalsData() );


  // pass normals to shader
  GLint n = glGetAttribLocation( m_shader.getShaderProgram(), "VertexNormal" );
  glEnableVertexAttribArray( n );
  glVertexAttribPointer( n, 3, GL_FLOAT, GL_FALSE, 0, 0 );

}

void GLWindow::numBoidsInput(const int _numBoids)
{
    // calls destructor for existing object and creates new flock object
    m_flock.reset(new Flock(this, _numBoids));
}
