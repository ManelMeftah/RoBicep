/********************************************************/
/*                     cube.cpp                                                 */
/********************************************************/
/*                Affiche a l'ecran un cube en 3D                      */
/********************************************************/

/* inclusion des fichiers d'en-tete freeglut */

#ifdef __APPLE__
#include <GLUT/glut.h> /* Pour Mac OS X */
#else
#include <GL/glut.h>   /* Pour les autres systemes */
#endif
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <armadillo>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/io.hpp>
#include <chrono>
#include <cmath>

#include "RoBiceps.h"

using namespace glm;
using namespace std;
using namespace arma;

//****************************************
#define NB_BRAS  4


mat4 rotationInterpolMatrice,rotationInterpolQuaternion;

//****************************************

glm::quat shoulderRotation = glm::angleAxis(0.0f, glm::vec3(0.0f, 1.0f, 0.0f));

glm::quat elbowRotation; // Quaternion pour la rotation du coude
float angleRotation = 0.0f; // Angle de rotation actuel
float deltaAngle = 0.1f; // Incrément de l'angle de rotation par itération


char presse;
int anglex,angley,x,y,xold,yold;

bool mouseLeftDown;
bool mouseRightDown;
bool mouseMiddleDown;
float mouseX, mouseY;
float cameraAngleX;
float cameraAngleY;
float cameraDistance=0.;

glm::vec3 target(0.,2.,0.);

/* Prototype des fonctions */
void affichage();
void clavier(unsigned char touche,int x,int y);
void reshape(int x,int y);
void idle();
void mouse(int bouton,int etat,int x,int y);
void mousemotion(int x,int y);

RoBiceps* robbie;

void anim( int NumTimer) ;

void initBrasRobot();

// mat4 interpolateMatrices(float t)
// {
//   mat4 m = (1-t)*startMatrix + t*endMatrix ;
//   return m;

// }
// mat4 interpolateQuaternions(float t)
// {

//   quat interpolatedquat = glm::slerp(q0, q1, t);
//   return mat4_cast(interpolatedquat);

// }

void initBrasRobot()
{
    float l0 = 1.0f;
    float l1 = 2.0f;
    float l2 = 1.0f;
    float l3 = 1.0f;

    float theta1 = 0.0f;
    float theta2 = -90.0f;
    float theta3 = 90.0f;
    float theta4 = 0.0f;

    quat q0 = angleAxis(radians(theta1), glm::vec3(0.0f, 0.0f, 1.0f));
    quat q1 = angleAxis(radians(theta2), glm::vec3(0.0f, 0.0f, 1.0f));
    quat q2 = angleAxis(radians(theta3), glm::vec3(0.0f, 0.0f, 1.0f));
    quat q3 = angleAxis(radians(theta4), glm::vec3(0.0f, 0.0f, 1.0f));

    mat4 t0 = translate(mat4(1.0f), glm::vec3(l0 / 2, 0.0f, 0.0f));
    mat4 t1 = translate(mat4(1.0f), glm::vec3(l1 / 2, 0.0f, 0.0f));
    mat4 t2 = translate(mat4(1.0f), glm::vec3(l2 / 2, 0.0f, 0.0f));
    mat4 t3 = translate(mat4(1.0f), glm::vec3(l3 / 2, 0.0f, 0.0f));

    vector<Bone> bones;
    bones.emplace_back(l0, theta1, glm::vec3(0.0, 0.0, 1.0), mat4_cast(q0), t0, glm::vec3(1.0, 0.0, 0.0), glm::vec3(0.0));
    bones.emplace_back(l1, theta2, glm::vec3(0.0, 0.0, 1.0), mat4_cast(q1), t1, glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0));
    bones.emplace_back(l2, theta3, glm::vec3(0.0, 0.0, 1.0), mat4_cast(q2), t2, glm::vec3(0.0, 0.0, 1.0), glm::vec3(0.0));
    bones.emplace_back(l3, theta4, glm::vec3(0.0, 0.0, 1.0), mat4_cast(q3), t3, glm::vec3(0.0, 1.0, 1.0), glm::vec3(0.0));

    robbie = new RoBiceps(bones);
    cout << "robbie is born!" << endl;
}




void anim( int NumTimer)
{
    using namespace std::chrono;
    static int i=0;
    static time_point<system_clock> refTime = system_clock::now()  ;

    time_point<system_clock> currentTime = system_clock::now(); // This and "end"'s type is std::chrono::time_point

    duration<double> deltaTime = currentTime - refTime;

    int deltaTemps = duration_cast<milliseconds>( deltaTime).count() ;


    float t = static_cast<float>(deltaTemps) / 1000.0f; //milliseconds to seconds
    // t = fmod(t, 1.0f);

    angleRotation += deltaAngle * t;
    angleRotation = glm::clamp(angleRotation, 0.0f, 45.0f);

    // cout << angleRotation << endl;

     rotationInterpolQuaternion = mat4_cast(glm::angleAxis(angleRotation, glm::vec3(0.0f, 1.0f, 0.0f)));

    glutPostRedisplay();
    glutTimerFunc(100,anim,1 );

    refTime = currentTime;

}

void inverseKinematics()
{
  int kmax = 100;
  double eps = 0.01;
  double lgMax = robbie->getLongueurTotale();

  arma::vec3 armaTarget = arma::vec3{target.x, target.y, target.z};

  arma::vec3 endPoint = robbie->computeEndPoint();

  float targetDotProduct = glm::dot(target, glm::vec3(1.0f, 0.0f, 0.0f));
  float targetNorm = arma::norm(armaTarget);
  double angleDegrees = glm::degrees(acos(targetDotProduct / targetNorm));

  arma::vec3 E = armaTarget - endPoint;

  arma::vec3 x_axis = arma::vec3{0, 0, 1};

  int k = 0;
  do
  {
    arma::mat J = robbie->calculeJacobienne(endPoint);
    arma::mat J_plus = arma::pinv(J);
    arma::vec V = J_plus * E;

    if(arma::max(V) > 2)
      V = V * 2 / arma::max(V);

    robbie->setArmPosition(V);
    endPoint = robbie->computeEndPoint();
    E = armaTarget - endPoint;

    k++;

  } while(k < kmax && arma::norm(E) > eps);

}


int main(int argc,char **argv)
{




  /* initialisation de glut et creation
     de la fenetre */
  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowPosition(200,200);
  glutInitWindowSize(1500,1500);
  glutCreateWindow("Robbie");

  /* Initialisation d'OpenGL */
  glClearColor(0.0,0.0,0.0,0.0);
  glColor3f(1.0,1.0,1.0);
  glPointSize(10.0);
  glEnable(GL_DEPTH_TEST);

  initBrasRobot();




  /* enregistrement des fonctions de rappel */
  glutDisplayFunc(affichage);
  glutKeyboardFunc(clavier);
  glutReshapeFunc(reshape);
  glutMouseFunc(mouse);
  glutMotionFunc(mousemotion);
  glutTimerFunc(200, anim, 1);

  glMatrixMode( GL_PROJECTION );
     glLoadIdentity();
   gluPerspective(60 ,1,.1,30.);


  /* Entree dans la boucle principale glut */
  glutMainLoop();

  delete robbie;
  return 0;
}

// void bras()
// {
//   float longueurSegment1 = 1.5f;
//   float longueurSegment2 = 1.0f;
//   float longueurSegment3 = 0.5f;

//   float angleSegment1 = 45.0f;
//   float angleSegment2 = 90.0f;
//   float angleSegment3 = 30.0f;

//   float scalYZ = 0.1;

//   float pos = 0.0;
//   pos += longueurSegment1/2;

// glPushMatrix();
//       glPushMatrix();
//           glTranslatef(pos, 0.0f, 0.0f);
//           glColor3f(1,0,0);
//           glScalef(longueurSegment1,scalYZ,scalYZ);
//           glutSolidCube(1.);
//       glPopMatrix();

//       pos += longueurSegment1 / 2 + longueurSegment2/2;

//       // Dessiner le segment 2
//       glPushMatrix();
//         glMultMatrixf(&rotationInterpolQuaternion[0][0]);
//         glTranslatef(pos, 0.0f, 0.0f);
//         glScalef(longueurSegment2,scalYZ,scalYZ);
//         glColor3f(0,1,0);
//         glutSolidCube(1.);
//       glPopMatrix();

//       glm::vec3 positionSegment3Relative = &rotationInterpolQuaternion[0][0] * glm::vec3(0.0f, longueurSegment2 / 2.0f, 0.0f);
//       glm::vec3 positionSegment3 = pos + positionSegment3Relative;


//       pos += longueurSegment2/2 + longueurSegment3 / 2;

//       // Dessiner le segment 3
//       glPushMatrix();
//         // glMultMatrixf(&shoulderRotation[0][0]);
//         glColor3f(0,0,1);
//         // glTranslatef(positionSegment3Relative);
//           glTranslatef(positionSegment3.x, positionSegment3.y, positionSegment3.z);
//         glScalef(longueurSegment3,scalYZ,scalYZ);
//         glutSolidCube(1.);
//       glPopMatrix();
//  glPopMatrix();
// }


// void bras() {
// // a animer pas interpolation de quaternions
//     glPushMatrix();
//     glMultMatrixf(&rotationInterpolQuaternion[0][0]);
//         glColor3f(1,0,0);
//         glScalef(2,.2,.2);
//         glTranslatef(.5,0.,0.);
//         glutSolidCube(1.);
//     glPopMatrix();

//     // a animer par interpolation de matrice
//     glPushMatrix();
//     glMultMatrixf(&rotationInterpolMatrice[0][0]);
//         glColor3f(1,1,0);
//         glScalef(2,.2,.2);
//         glTranslatef(.5,0.,0.);
//         glutSolidCube(1.);
//     glPopMatrix();


//     glPopMatrix();
// }

void afficheRepere()
{
  //Repère
    //axe x en rouge
    glBegin(GL_LINES);
        glColor3f(1.0,0.0,0.0);
    	glVertex3f(0, 0,0.0);
    	glVertex3f(1, 0,0.0);
    glEnd();
    //axe des y en vert
    glBegin(GL_LINES);
    	glColor3f(0.0,1.0,0.0);
    	glVertex3f(0, 0,0.0);
    	glVertex3f(0, 1,0.0);
    glEnd();
    //axe des z en bleu
    glBegin(GL_LINES);
    	glColor3f(0.0,0.0,1.0);
    	glVertex3f(0, 0,0.0);
    	glVertex3f(0, 0,1.0);
    glEnd();
}

void displayTarget()
{
  glPushMatrix();
      glTranslatef(target.x,target.y,target.z);
      glScalef(.5, 0.5, .5);
      afficheRepere();
  glPopMatrix();
}


void affichage()
{
  int i,j;
  /* effacement de l'image avec la couleur de fond */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
glShadeModel(GL_SMOOTH);
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glTranslatef(0.,0.,-5.);
  glRotatef(angley,1.0,0.0,0.0);
  glRotatef(anglex,0.0,1.0,0.0);

  afficheRepere();


    // bras();
    vector<Bone> bones = robbie->getBones();

    // for (const auto& os : robbie->getBones()) {
    //     os.display();
    // }
    inverseKinematics();

    robbie->display();

    displayTarget();



  glFlush();

  //On echange les buffers
  glutSwapBuffers();
}

void clavier(unsigned char touche,int x,int y)
{
  switch (touche)
    {
    case '1':
//    orientation[0]+=.5;
    glutPostRedisplay();
    break;
//  case '&':
//  orientation[0]-=.5;
//  glutPostRedisplay();
//  break;
  case '2':
//  orientation[1]+=.5;
  glutPostRedisplay();
  break;
//  case 'é':
//  orientation[1]+=.5;
//  glutPostRedisplay();
//  break;

  case 'p': /* affichage du carre plein */
      glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
      glutPostRedisplay();
      break;
    case 'f': /* affichage en mode fil de fer */
      glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
      glutPostRedisplay();
      break;
    case 's' : /* Affichage en mode sommets seuls */
      glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
      glutPostRedisplay();
      break;
    case 'd':
      glEnable(GL_DEPTH_TEST);
      glutPostRedisplay();
      break;
    case 'D':
      glDisable(GL_DEPTH_TEST);
      glutPostRedisplay();
      break;
    case 'q' : /*la touche 'q' permet de quitter le programme */
      exit(0);
    }
}

void reshape(int x,int y)
{
  if (x<y)
    glViewport(0,(y-x)/2,x,x);
  else
    glViewport((x-y)/2,0,y,y);
}

void mouse(int button, int state,int x,int y)
{
  /* si on appuie sur le bouton gauche */
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
  {
    presse = 1; /* le booleen presse passe a 1 (vrai) */
    xold = x; /* on sauvegarde la position de la souris */
    yold=y;
  }
  /* si on relache le bouton gauche */
  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    presse=0; /* le booleen presse passe a 0 (faux) */
}

void mousemotion(int x,int y)
  {
    if (presse) /* si le bouton gauche est presse */
    {
      /* on modifie les angles de rotation de l'objet
	 en fonction de la position actuelle de la souris et de la derniere
	 position sauvegardee */
      anglex=anglex+(x-xold);
      angley=angley+(y-yold);
      glutPostRedisplay(); /* on demande un rafraichissement de l'affichage */
    }
//    else
//    {
//        pointCible.x= ;
//        pointCible.y= ;
//    }

    xold=x; /* sauvegarde des valeurs courante de le position de la souris */
    yold=y;
  }
