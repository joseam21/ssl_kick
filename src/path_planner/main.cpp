#include <iostream>
#include <QApplication>
#include "Visualizer.h"
int main(int argc, char **argv) {
    std::cout << "END HERE" << std::endl;
    QApplication a(argc, argv);
    Visualizer w;
    w.resize(810,700);
    w.show();
    return a.exec();
}
