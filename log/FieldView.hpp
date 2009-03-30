// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#ifndef FIELDVIEW_HPP_
#define FIELDVIEW_HPP_

#include <QGLWidget>
#include <QVector>

#include <Team.h>
#include <LogFrame.hpp>
#include <framework/Module.hpp>

namespace Log
{
	/** class that performs drawing of log data onto the field */
	class FieldView : public QGLWidget
	{
		Q_OBJECT;

		public:
			FieldView(QWidget* parent = 0);
			
			void team(Team t);
			
			void addModule(Module* module);

		protected:
			void paintEvent(QPaintEvent* pe);
			void resizeEvent(QResizeEvent* re);
			
			void mouseReleaseEvent(QMouseEvent*);
			void mousePressEvent(QMouseEvent*);
			void mouseMoveEvent(QMouseEvent*);
			
		private:
			/** convert from screen space to team space */
			Geometry::Point2d toTeamSpace(int x, int y) const;
			
			void drawField(QPainter&);

		public Q_SLOTS:
			void frame(Packet::LogFrame* frame);

		private:
			/** frame to display */
			Packet::LogFrame* _frame;

			//translations for placing robots in team space
			float _tx, _ty, _ta;

			Team _team;
			
			//list of modules for fieldOverlay hook
			QVector<Module*> _modules;


	};
}

#endif /* FIELDVIEW_HPP_ */
