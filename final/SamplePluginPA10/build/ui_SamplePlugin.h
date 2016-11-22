/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCalendarWidget>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;

    QPushButton *_btn0;
    QPushButton *_btn1;

    QCheckBox *_checkBox;

    QSlider *_slider_0;
    QSlider *_slider_1;
    QSlider *_slider_2;

    QLabel *_cameraView;
    QLabel *_cvView;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if( SamplePlugin->objectName().isEmpty() ){
        	SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        }
        SamplePlugin->resize(400, 600);

        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));

        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));

        //	Buttons
        QGridLayout * gridForButtons = new QGridLayout(dockWidgetContents);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));
        gridForButtons->addWidget(_btn0,0,0,1,1,Qt::AlignVCenter | Qt::AlignLeft);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QString::fromUtf8("_btn1"));
        gridForButtons->addWidget(_btn1,0,1,1,1,Qt::AlignVCenter | Qt::AlignRight);

        verticalLayout->addLayout(gridForButtons);

        //	Checkboxes
        _checkBox = new QCheckBox(dockWidgetContents);
        _checkBox->setObjectName(QString::fromUtf8("_checkBox"));
        verticalLayout->addWidget(_checkBox);

        //	Sliders
        _slider_0 = new QSlider(dockWidgetContents);
        _slider_0->setObjectName(QString::fromUtf8("_slider_0"));
        _slider_0->setOrientation(Qt::Horizontal);
        verticalLayout->addWidget(_slider_0);

		_slider_1 = new QSlider(dockWidgetContents);
		_slider_1->setObjectName(QString::fromUtf8("_slider_1"));
		_slider_1->setOrientation(Qt::Horizontal);
		verticalLayout->addWidget(_slider_1);

		_slider_2 = new QSlider(dockWidgetContents);
		_slider_2->setObjectName(QString::fromUtf8("_slider_2"));
		_slider_2->setOrientation(Qt::Horizontal);
		verticalLayout->addWidget(_slider_2);

		//	Camera views
		QGridLayout * gridForViews = new QGridLayout(dockWidgetContents);

        _cameraView = new QLabel(dockWidgetContents);
        _cameraView->setObjectName(QString::fromUtf8("_label"));
        gridForViews->addWidget(_cameraView,0,0,1,1,Qt::AlignVCenter | Qt::AlignLeft);

        _cvView = new QLabel(dockWidgetContents);
		_cvView->setObjectName(QString::fromUtf8("_label"));
		gridForViews->addWidget(_cvView,0,1,1,1,Qt::AlignVCenter | Qt::AlignRight);

        verticalLayout->addLayout(gridForViews);
        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        _btn0->setText(QApplication::translate("SamplePlugin", "Button 0", 0, QApplication::UnicodeUTF8));
        _btn1->setText(QApplication::translate("SamplePlugin", "Button 1", 0, QApplication::UnicodeUTF8));
        _checkBox->setText(QApplication::translate("SamplePlugin", "CheckBox", 0, QApplication::UnicodeUTF8));
        _cameraView->setText(QApplication::translate("SamplePlugin", "Camera view", 0, QApplication::UnicodeUTF8));
        _cameraView->setText(QApplication::translate("SamplePlugin", "Computer view", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
