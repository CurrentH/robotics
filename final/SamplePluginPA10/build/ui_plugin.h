/********************************************************************************
** Created by: Qt User Interface Compiler version 4.8.7
********************************************************************************/

#ifndef UI_PLUGIN_H
#define UI_PLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
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

class Ui_Plugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;

    QPushButton *_btnStart;
    QPushButton *_btnStop;
    QPushButton *_btnRestart;

    QComboBox *_ddMarker;
	QComboBox *_ddSequence;

    QCheckBox *_checkBox;

    QSlider *_slider_0;
    QSlider *_slider_1;
    QSlider *_slider_2;

    QLabel *_cameraView;
    QLabel *_cvView;

    void setupUi(QDockWidget *Plugin)
    {
        if( Plugin->objectName().isEmpty() ){
        	Plugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        }
        Plugin->resize(400, 600);

        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));

        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));

        //	Buttons
        QGridLayout * gridForStuff = new QGridLayout(dockWidgetContents);

        _btnStart = new QPushButton(dockWidgetContents);
        _btnStart->setObjectName(QString::fromUtf8("Start"));
        gridForStuff->addWidget(_btnStart, 0, 0, 1, 1);

        _btnStop = new QPushButton(dockWidgetContents);
        _btnStop->setObjectName(QString::fromUtf8("Stop"));
        gridForStuff->addWidget(_btnStop, 0, 1, 1, 1);

        _btnRestart = new QPushButton(dockWidgetContents);
        _btnRestart->setObjectName(QString::fromUtf8("Restart"));
		gridForStuff->addWidget(_btnRestart, 0, 2, 1, 1);

        //	Drop down bars
		_ddMarker = new QComboBox(dockWidgetContents);
		_ddMarker->setObjectName(QString::fromUtf8("ddMarker"));
		gridForStuff->addWidget(_ddMarker, 1, 0, 1, 1);

		_ddSequence = new QComboBox(dockWidgetContents);
		_ddSequence->setObjectName(QString::fromUtf8("ddMarker"));
		gridForStuff->addWidget(_ddSequence, 1, 2, 1, 1);

        verticalLayout->addLayout(gridForStuff);

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

        Plugin->setWidget(dockWidgetContents);

        retranslateUi(Plugin);

        QMetaObject::connectSlotsByName(Plugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", 0, QApplication::UnicodeUTF8));

        _btnStart->setText(QApplication::translate("SamplePlugin", "Start", 0, QApplication::UnicodeUTF8));
        _btnStop->setText(QApplication::translate("SamplePlugin", "Stop", 0, QApplication::UnicodeUTF8));
        _btnRestart->setText(QApplication::translate("SamplePlugin", "Restart", 0, QApplication::UnicodeUTF8));

        _ddMarker->clear();
        _ddMarker->insertItems(0, QStringList()
		 << QApplication::translate("SamplePlugin", "Marker1.ppm", 0, QApplication::UnicodeUTF8)
		 << QApplication::translate("SamplePlugin", "Marker2a.ppm", 0, QApplication::UnicodeUTF8)
		 << QApplication::translate("SamplePlugin", "Marker2b.ppm", 0, QApplication::UnicodeUTF8)
		 << QApplication::translate("SamplePlugin", "Marker3.ppm", 0, QApplication::UnicodeUTF8)
		);

		_ddSequence->clear();
		_ddSequence->insertItems(0, QStringList()
		 << QApplication::translate("SamplePlugin", "MarkerMotionSlow.txt", 0, QApplication::UnicodeUTF8)
		 << QApplication::translate("SamplePlugin", "MarkerMotionMedium.txt", 0, QApplication::UnicodeUTF8)
		 << QApplication::translate("SamplePlugin", "MarkerMotionFast.txt", 0, QApplication::UnicodeUTF8)
		);

        _checkBox->setText(QApplication::translate("SamplePlugin", "CheckBox", 0, QApplication::UnicodeUTF8));

        _cameraView->setText(QApplication::translate("SamplePlugin", "Camera view", 0, QApplication::UnicodeUTF8));
        _cameraView->setText(QApplication::translate("SamplePlugin", "Computer view", 0, QApplication::UnicodeUTF8));
    }
};

namespace Ui {
    class Plugin: public Ui_Plugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLUGIN_H
