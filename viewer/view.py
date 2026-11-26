#!/usr/bin/env python3
"""Entry point: validate data before importing any graphical dependency."""
import argparse
from pathlib import Path
import sys
import warnings


def main():
    parser = argparse.ArgumentParser(description='Visualizzatore UAV 3D offline · ENU · quaternion Hamilton')
    parser.add_argument('csv', nargs='?', type=Path, help='CSV della simulazione; se omesso apre una finestra di selezione')
    parser.add_argument('--config', type=Path, help='Override TOML (default relativo al viewer)')
    parser.add_argument('--validate', action='store_true', help='Verifica il CSV senza avviare Qt/VTK')
    parser.add_argument('--screenshot', type=Path, metavar='PNG', help='Rendering offscreen di una posa ed esci')
    parser.add_argument('--time', type=float, help='Tempo della posa iniziale o dello screenshot [s]')
    args = parser.parse_args()
    if (args.validate or args.screenshot) and args.csv is None:
        parser.error('Specificare un CSV per --validate o --screenshot')
    try:
        from uav_viewer.config import load_config
        from uav_viewer.data import load_csv
        config = load_config(args.config)
        qt_app = None
        if args.csv is None:
            import os
            os.environ['QT_API'] = 'pyside6'
            from PySide6 import QtWidgets
            qt_app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(None, 'Apri una simulazione', '', 'CSV (*.csv)')
            if not filename:
                return 0
            args.csv = Path(filename)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            flight = load_csv(args.csv, config)
        notices = [str(item.message) for item in caught]
        for notice in notices:
            print(f'Avviso: {notice}', file=sys.stderr)
        if args.validate:
            print(f'OK: {args.csv}\n{len(flight.times)} campioni; t={flight.start:g}…{flight.end:g} s; {len(flight.gaps)} lacune')
            return 0
        initial_time = flight.start if args.time is None else args.time
        pose = flight.sample(initial_time)
        if args.screenshot:
            import pyvista as pv
            from uav_viewer.scene import FlightScene
            plotter = pv.Plotter(off_screen=True, window_size=(1440, 900))
            try:
                scene = FlightScene(plotter, flight, config)
                scene.update(pose)
                scene.set_view('iso')
                plotter.screenshot(str(args.screenshot))
            finally:
                plotter.close()
            print(f'Screenshot: {args.screenshot}')
            return 0
        import os
        os.environ['QT_API'] = 'pyside6'
        from PySide6 import QtCore, QtWidgets
        from shiboken6 import isValid
        from uav_viewer.app import ViewerWindow
        qt_app = qt_app or QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
        window = ViewerWindow(flight, config, args.csv, notices)
        qt_app.aboutToQuit.connect(window.shutdown)
        try:
            window.player.seek(initial_time)
            window.update_frame()
            window.show()
            return qt_app.exec()
        finally:
            # Do not leave live widgets for PySide's Python-exit cleanup:
            # their parents and OpenGL context may already be gone by then.
            if isValid(window):
                window.close()
                QtCore.QCoreApplication.sendPostedEvents(window, QtCore.QEvent.Type.DeferredDelete)
    except ImportError as exc:
        print(f'Dipendenza mancante: {exc}\nInstalla: python -m pip install -r viewer/requirements.txt', file=sys.stderr)
        return 2
    except (OSError, ValueError) as exc:
        print(f'Errore: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
