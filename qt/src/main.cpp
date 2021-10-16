#include "pcd_view_qt.h"

#include <QApplication>
#include <QSurfaceFormat>

// #include <QVTKOpenGLNativeWidget.h>

int main(int argc, char* argv[])
{
    // QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication a(argc, argv);
    PCDViewQt w;
    w.show();

    return a.exec();
}
