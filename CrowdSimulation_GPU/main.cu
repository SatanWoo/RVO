//
//  main.cpp
//  Crowd Simulation
//
//  Created by z on 15-3-26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//


#include <gl/glut.h>
#include <iostream>
#include "Vector.h"
#include "MapController.h"

using namespace std;

const double ScreenWidth = 800;
const double ScreenHeight = 448;
const double MapWidth = 25;
const double MapHeight = 14;

MapController *mapController = NULL;

void glRender()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glLoadIdentity();

	mapController->render();
	glutSwapBuffers();
}

void glIdle()
{
	int n = 1;
	while (n--) {
		mapController->update();
	}

	glutPostRedisplay();
}

void glMouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		int xPos = x / MapController::MapGridSize;
		int yPos = MapHeight -  y / MapController::MapGridSize;

		cout << xPos << ":" << yPos << endl;

		mapController->updateDestinationPoint(b2Vec2(xPos, yPos));
		break;
	}
}

int main(int argc, const char * argv[])
{
	glutInit(&argc, const_cast<char **>(argv));
	glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
	glutInitWindowSize(ScreenWidth, ScreenHeight);
	glutCreateWindow("Crowd Simulation");

	glutDisplayFunc(glRender);
	glutIdleFunc(glIdle);
	glutMouseFunc(glMouse);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, MapWidth * MapController::MapGridSize, 0, MapHeight * MapController::MapGridSize);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_POINT_SMOOTH);
	//glEnable(GL_BLEND);

	mapController = new MapController(MapWidth, MapHeight, MapHeight * 2);

	glutMainLoop();

	delete mapController;
	mapController = NULL;
	return EXIT_SUCCESS;
}