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
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QPushButton *pushButton;
    QPushButton *_btn0;
    QPushButton *_btn1;
    QComboBox *comboBox;
    QComboBox *comboBox_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout_2;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(428, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pushButton = new QPushButton(dockWidgetContents);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 0, 2, 1, 1);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));

        gridLayout->addWidget(_btn0, 0, 0, 1, 1);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QString::fromUtf8("_btn1"));

        gridLayout->addWidget(_btn1, 0, 1, 1, 1);

        comboBox = new QComboBox(dockWidgetContents);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout->addWidget(comboBox, 1, 0, 1, 1);

        comboBox_2 = new QComboBox(dockWidgetContents);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));

        gridLayout->addWidget(comboBox_2, 1, 2, 1, 1);


        verticalLayout->addLayout(gridLayout);


        verticalLayout_2->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));

        horizontalLayout->addLayout(gridLayout_2);


        verticalLayout_3->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(verticalLayout_3);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("SamplePlugin", "PushButton", 0, QApplication::UnicodeUTF8));
        _btn0->setText(QApplication::translate("SamplePlugin", "PushButton0", 0, QApplication::UnicodeUTF8));
        _btn1->setText(QApplication::translate("SamplePlugin", "PushButton1", 0, QApplication::UnicodeUTF8));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
        );
        comboBox_2->clear();
        comboBox_2->insertItems(0, QStringList()
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "New Item", 0, QApplication::UnicodeUTF8)
        );
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
