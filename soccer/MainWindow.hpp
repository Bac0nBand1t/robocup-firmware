// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QString>

#include <Team.h>

#include <log/FieldView.hpp>
#include <log/LogFile.hpp>
#include <log/LogControl.hpp>
#include <log/TreeView.hpp>
#include <log/TreeModel.hpp>

#include "ConfigFileTab.hpp"
#include "Processor.hpp"
#include "ui_MainWindow.h"

class PlayConfigTab;

class MainWindow : public QMainWindow
{
	Q_OBJECT;
	
	public:
		MainWindow(Team t, QString filename);
		~MainWindow();
		
		Processor *processor()
		{
			return &_processor;
		}
		
		PlayConfigTab *playConfig() const;
		
	private:
		Ui_MainWindow ui;
		
		Team _team;
	
		Processor _processor;
	
		Log::TreeModel* _treeModel;
	
		Log::LogFile* _logFile;
	
		QString _configFile;

		ConfigFileTab* _configFileTab;
};
