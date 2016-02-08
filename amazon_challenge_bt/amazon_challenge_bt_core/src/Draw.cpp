#include<amazon_challenge_bt_core/Draw.h>
// Display callback ------------------------------------------------------------


//    enum Node {PARALLEL, SELECTOR, SEQUENCE, SEQUENCESTAR, SELECTORSTAR, ACTION, CONDITION,EMPTY};
void draw_status(float x, float y, int node_status)
{
/*

    glClear(GL_COLOR_BUFFER_BIT);   // Erase everything

    double STATUS_WIDTH = 0.1;
    double STATUS_HEIGHT = 0.1;

    switch (1)
    {
    case 1:    glColor3f(0.0, 1.0, 0.0); break;
    case 2:    glColor3f(1.0, 0.0, 0.0); break;
    case 3:    glColor3f(1.0, 1.0, 0.0); break;
    case 4:    glColor3f(1.0, 1.0, 1.0); break;
    default: break;
    }

    // important

    glBegin(GL_LINES);
    glVertex2d(x - STATUS_WIDTH, y + STATUS_HEIGHT);
    glVertex2d(x + STATUS_WIDTH, y + STATUS_HEIGHT);
    glVertex2d(x + STATUS_WIDTH, y + STATUS_HEIGHT);
    glVertex2d(x + STATUS_WIDTH, y - STATUS_HEIGHT);
    glVertex2d(x + STATUS_WIDTH, y - STATUS_HEIGHT);
    glVertex2d(x - STATUS_WIDTH, y - STATUS_HEIGHT);
    glVertex2d(x - STATUS_WIDTH, y - STATUS_HEIGHT);
    glVertex2d(x - STATUS_WIDTH, y + STATUS_HEIGHT);
    glVertex2d(x - STATUS_WIDTH, y + STATUS_HEIGHT);
    glVertex2d(x + STATUS_WIDTH, y + STATUS_HEIGHT);
    glEnd();

    glutSwapBuffers();
*/
}



void drawString (void * font, char *s, float x, float y, float z){
     unsigned int i;
     glRasterPos3f(x, y, z);

     for (i = 0; i < 2; i++)
          glutBitmapCharacter (font, s[i]);
}


// draw the node itself using a solid square (color coded)



void renderBitmapString(float x, float y, void *font,const char *string){
    const char *c;
    glRasterPos2f(x, y);
    for (c=string; *c != '\0'; c++) {
        glutBitmapCharacter(font, *c);
    }
}



void draw_node(float x, float y, int node_type, const char *leafName, int status)
{
    void * font = GLUT_BITMAP_8_BY_13;

    float NODE_WIDTH = 0.02;
    float NODE_HEIGHT = 0.02;
    switch (node_type)
    {
    case BT::SELECTORSTAR:
        drawString(font, "?*", (x + NODE_WIDTH - 0.035), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::SEQUENCESTAR:
        drawString(font, ">*", (x - NODE_WIDTH + 0.01 ), (y - NODE_HEIGHT/2), 0);
        break;

    case BT::SELECTOR:
        drawString(font, "?", (x + NODE_WIDTH - 0.025), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::SEQUENCE:
        drawString(font, ">", (x - NODE_WIDTH + 0.015), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::PARALLEL:
        drawString(font, "=", (x - NODE_WIDTH + 0.01), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::DECORATOR:
        drawString(font, "D", (x - NODE_WIDTH + 0.01), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::ACTION:
       {
            std::string st(leafName,0, 15);
            NODE_WIDTH = 0.0;
            for (unsigned int i = 0; i < st.size(); i++)
              NODE_WIDTH +=  0.01;
        }
        renderBitmapString((x - NODE_WIDTH +0.02), (y - NODE_HEIGHT/2), font,leafName);
        glColor3f(0.2, 1.0, 0.2);
        break;
    case BT::CONDITION:
    {
         std::string st(leafName,0, 15);
         NODE_WIDTH = 0.0;
         for (unsigned int i = 0; i < st.size(); i++)
           NODE_WIDTH +=  0.01;
     }
        renderBitmapString((x - NODE_WIDTH +0.02), (y - NODE_HEIGHT/2), font,leafName);
        break;
    default: break;
    }

    switch (status)
    {
    case RUNNING:   glColor3f(0.8, 0.8, 0.8); break;
    case SUCCESS:   glColor3f(0.0, 1.0, 0.0); break;
    case FAILURE:   glColor3f(1.0, 0.0, 0.0); break;
    case IDLE:   glColor3f(0.0, 0.0, 0.0); break;
    default: break;
    }


    glBegin(GL_LINE_LOOP);
    glVertex3f((GLfloat) (x + NODE_WIDTH), (GLfloat) (y - NODE_HEIGHT), (GLfloat) 0.0);
    glVertex3f((GLfloat) (x + NODE_WIDTH), (GLfloat) (y + NODE_HEIGHT), (GLfloat) 0.0);
    glVertex3f((GLfloat) (x - NODE_WIDTH), (GLfloat) (y + NODE_HEIGHT), (GLfloat) 0.0);
    glVertex3f((GLfloat) (x - NODE_WIDTH), (GLfloat) (y - NODE_HEIGHT), (GLfloat) 0.0);
    glColor3f(0.0, 0.0, 0.0);
    glEnd();
}

// draw the edge connecting one node to the other
void draw_edge(GLfloat parent_x, GLfloat parent_y, GLfloat parent_size, GLfloat child_x, GLfloat child_y, GLfloat child_size)
{
    glLineWidth(1.5);
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(parent_x, parent_y-parent_size, 0.0);
    glVertex3f(child_x, child_y+child_size, 0);
    glEnd();
}



// Keyboard callback function ( called on keyboard event handling )
void keyboard(unsigned char key, int x, int y)
{
    if (key == 'q' || key == 'Q')
        exit(EXIT_SUCCESS);
}


void resize(int width, int height) {
        // we ignore the params and do:
    glutReshapeWindow(1024, 768);
}





void drawCircle(float radius)
{
   glBegin(GL_LINE_LOOP);

   for (int i=0; i<= 360; i++)
   {

      float degInRad = i*3.14142/180;
      glVertex2f(cos(degInRad)*radius,sin(degInRad)*radius);
   }

   glEnd();
}

/*
void display()
{



    // clear the draw buffer .
    glClear(GL_COLOR_BUFFER_BIT);   // Erase everything


    GLfloat x_seq1 = 0.0;
    GLfloat y_seq1 = 0.4;

    GLfloat x_con1 = x_seq1 - 0.2;
    GLfloat y_con1 = y_seq1 - 0.2;

    GLfloat x_act1 = x_seq1 + 0.2;
    GLfloat y_act1 = y_seq1 - 0.2;


    draw_edge(x_seq1, y_seq1, 0.02, x_con1, y_con1, 0.02);
    draw_edge(x_seq1, y_seq1, 0.02, x_act1, y_act1, 0.02);

    draw_node((GLfloat) x_seq1, (GLfloat) y_seq1,SEQUENCE, "seq1", FAILURE);

    draw_node((GLfloat) x_con1, (GLfloat) y_con1,CONDITION, "Condition 1", SUCCESS);
    draw_node((GLfloat) x_act1, (GLfloat) y_act1,ACTION, "Action 1", RUNNING);

    glutSwapBuffers();

}




// Main execution  function
void drawTree()
{

    int argc = 1;
    char *argv[1] = {(char*)"Something"};

    glutInit(&argc, argv);      // Initialize GLUT
    glutInitWindowSize(1024,1024);

    glutCreateWindow("Behavior Tree");  // Create a window
    glutReshapeFunc(resize);
    glClearColor( 1, 1, 1, 0.5);
    glutDisplayFunc(display);   // Register display callback


    glutKeyboardFunc(keyboard); // Register keyboard callback
    glutMainLoop();             // Enter main event loop
}



*/
