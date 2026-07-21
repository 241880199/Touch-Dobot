// GLUT 配置验证 — 弹出一个带旋转线框立方体的窗口即表示配置成功
// 编译: 将此文件加入 .vcxproj，或单独用 cl 编译
// 运行: 把 glut32.dll 放到 exe 同目录

#include <GL/glut.h>
#include <cmath>

float angle = 0.0f;

void display() {
    glClearColor(0.1f, 0.12f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 1.0, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 3, 8, 0, 0, 0, 0, 1, 0);

    glRotatef(angle, 0, 1, 0);

    // 三色坐标轴
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X 轴 (红)
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(2, 0, 0);
    // Y 轴 (绿)
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 2, 0);
    // Z 轴 (蓝)
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 2);
    glEnd();

    // 线框立方体
    glColor3f(1.0f, 0.2f, 0.2f);
    glutWireCube(1.5);

    // 文字
    glColor3f(1, 1, 1);
    glRasterPos3f(-0.5f, 1.5f, 0);
    const char* msg = "GLUT OK!";
    for (const char* c = msg; *c; c++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

    glutSwapBuffers();
}

void idle() {
    angle += 0.5f;
    if (angle > 360.0f) angle -= 360.0f;
    glutPostRedisplay();
}

int main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("GLUT Config Test - Touch_Client");

    glEnable(GL_DEPTH_TEST);

    glutDisplayFunc(display);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
