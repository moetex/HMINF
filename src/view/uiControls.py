"""
UI-Integration für VPython
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Callable, Dict

from vpython import checkbox, canvas, color, slider, menu, button, rate, wtext

#from src.controller.programController import Simulation


# ----------------------------------------
# Ui Style Manager
# ----------------------------------------

class UIStyleManager:
    """Verwaltet CSS-Styles für die UI"""

    @staticmethod
    def get_slidebar_style() -> str:
        """ Gibt CSS für die rechte Sidebar zurück """
        return """
        <style>
            .control-panel {
                position: fixed;
                right: 20px;
                top: 100px;
                width: 320px;
                max-height: calc(100vh - 120px);
                overflow-y: auto;
                background: linear-gradient(135deg, #f5f7fa 0%, #c3cfc2 100%);
                padding: 20px;
                border-radius: 12px;
                box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;              
            }
            
            .ui-section {
                top: 0px;
                margin-bottom: 20px;
                padding: 10px;
                background: rgba(255, 255, 255, 0.7);
                border-radius: 8px;
                backdrop-filter: blur(10px);
                display: contents;            
            }
            
            .ui-section:last-child {
                margin-bottom: 0;
            }
            
            .ui-section h3 {     
                background-color: white;      
                margin: 0 0 12px 0;
                color: #2c3e50;
                font-size: 16px;
                font-weight: 600;
                border-bottom: 2px solid #3498db;
                padding-bottom: 6px;                
            }

            .ui-slider {
                margin-top: 5px !important;
                margin-bottom: 15px !important;
            }
            
            .control-label {
                display: block;
                margin: 10px 0 5px 0;
                color: #34495e;
                font-size: 13px;
                font-weight: 500;            
            }
            
            .value-display {
                float: right;
                color: #3498db;
                font-weight: 700;
                font-size: 14px;            
            }
            
            .checkbox-wrapper {
                margin: 10px 0;
                display: flex;
                align-items: center;
                padding: 8px;
                background: rgba(255, 255, 255, 0.5);
                border-radius: 4px;
                transition: background 0.2s;            
            }
            
            .checkbox-wrapper:hover {
                background: rgba(255, 255, 255, 0.8);
            }
            
            .checkbox-wrapper label {
                color: #2c3e50;
                font-size: 13px;
                cursor: pointer;
                user-select: none;
            }
            
            select {
                width: 100%;
                max-width: 240px;
                padding: 8px;
                margin-bottom: 8px;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                background: white;
                color: #2c3e50;
                font-size: 13px;
                overflow: hidden;
                text-overflow: ellipsis;                
            }
            
            input[type="range"] {
                width: 100%;
                margin: 5px 0 15px 0;
            }
            
            .btn-group {
                display: grid !important;
                grid-template-columns: 1fr 1fr !important;
                gap: 8px;
                margin: 15px 0 !important;
            }
            
            .info-box {
                background: #e8f4f8;
                border-left: 4px solid #3498db;
                padding: 10px;
                margin-top: 10px;
                border-radius: 4px;
                font-size: 12px;
                color: #2c3e50;
            }
               
            
            #glowscript div {
                white-space: normal !important;
            }
            
            
            
        </style>
        """


# ----------------------------------------
# Base UI Component
# ----------------------------------------

class UIComponent(ABC):
    """Abstrakte Basisklasse für UI-Komponenten"""

    def __init__(self, scene_ref):
        self.scene = scene_ref
        self.enabled = True

    @abstractmethod
    def render(self):
        """Rendert die Komponente"""
        pass

    def set_enabled(self, enable: bool):
        """Aktiviert/Deaktiviert die Komponente"""
        self.enabled = enable


# ----------------------------------------
# Base UI Component
# ----------------------------------------

@dataclass
class SliderConfig:
    """Konfiguration für einen Slider"""
    label: str
    min_val: float
    max_val: float
    initial_val: float
    step: float
    unit: str = ""
    value_id: Optional[str] = None
    callback: Optional[Callable] = None

class UISlider(UIComponent):
    """Slider-Koponente mit Label und Wert-Anzeige"""

    def __init__(self, scene_ref, config: SliderConfig):
        super().__init__(scene_ref)
        self.config = config
        self.slider_widget = None
        self.value_text = None
        #self.value_id = config.value_id or f"val_{id(self)}"
        self._current_value = config.initial_val

    def render(self):
        """Rendert den Slider mit Label"""
        self.scene.append_to_caption(f"""
        <label class='control-label'>
            {self.config.label}
        </label>
        """)

        # Wert-Anzeige
        self.value_text = wtext(text=f"{self._format_value(self.config.initial_val)} {self.config.unit}")

        self.scene.append_to_caption("<br>")

        self.slider_widget = slider(
            min=self.config.min_val,
            max=self.config.max_val,
            value=self.config.initial_val,
            step=self.config.step,
            bind=self._on_change
            #,canvas=self.scene   # TODO
        )

        self.scene.append_to_caption("<br>")
        return self

    def _on_change(self, evt):
        """Interner Callback für Slider-Änderungen"""
        self._current_value = evt.value
        self.update_display(evt.value)

        if self.config.callback and self.enabled:
            try:
                self.config.callback(evt.value)
            except Exception as e:
                print(f"Fehler in Slider-Callback: {e}")

    def update_display(self, value: float):
        """Aktualisiert die Wert-Anzeige"""
        if self.value_text:
            formatted = self._format_value(value)
            self.value_text.text = f"{formatted} {self.config.unit}"

        #self.scene.append_to_caption(f"""
        #<script>
        #    document.getElementById('{self.value_id}').textContent = '{self._format_value(value)}');
        #</script>
        #""")

    def _format_value(self, value: float) -> str:
        """Formatiert den Wert für die Anzeige"""
        if self.config.step >= 1:
            return f"{value:.0f}"
        elif self.config.step >= 0.1:
            return f"{value:.0f}"
        else:
            return f"{value:.2f}"

    @property
    def value(self) -> float:
        """Gibt den aktuellen Wert zurück"""
        return self._current_value

    def set_value(self, value: float):
        """Setzt den Slider-Wert programmiert"""
        if self.slider_widget:
            self.slider_widget.value = value
            self._current_value = value
            self.update_display(value)

    def set_enabled(self, enabled: bool):
        """Aktiviert/Deaktiviert den Slider"""
        super().set_enabled(enabled)

class UICheckbox(UIComponent):
    """Checkbox-Komponente"""

    def __init__(self, scene_ref, label: str, checked: bool = True,
                 callback: Optional[Callable] = None):
        super().__init__(scene_ref)
        self.label = label
        self.checked = checked
        self.callback = callback
        self.checkbox_widget = None

    def render(self):
        """Render die Checkbox"""
        self.scene.append_to_caption("<div class='checkbox-wrapper'>")
        self.checkbox_widget = checkbox(
            text=self.label,
            checked=self.checked,
            bind=self._on_change
            #,canvas=self.scene   #TODO
        )
        self.scene.append_to_caption("</div>")
        return self

    def _on_change(self, evt):
        """Interner Callback für Checkbox-Änderungen"""
        self.checked = evt.checked
        if self.callback:
            self.callback(evt.checked)

    @property
    def is_checked(self) -> bool:
        return self.checked


class UIMenu(UIComponent):
    """Dropdown-Menü Komponente"""

    def __init__(self, scene_ref, label: str, choices: list,
                 selected: Optional[str] = None,
                 callback: Optional[Callable] = None):
        super().__init__(scene_ref)
        self.label = label
        self.choices = choices
        self.selected = selected or (choices[0] if choices else None)
        self.callback = callback
        self.menu_widget = None

    def render(self):
        """Rendert das Menü"""
        self.scene.append_to_caption(f"<label class='control-label'>{self.label}</label>")
        self.menu_widget = menu(
            choices=self.choices,
            selected=self.selected,
            bind=self._on_change,
            width=240,
            canvas=self.scene   #TODO
        )
        self.scene.append_to_caption("<br>")
        return self

    def _on_change(self, evt):
        """Interner Callback für Menü-Änderungen"""
        self.selected = evt.selected
        if self.callback:
            self.callback(evt.selected)

    def get_selected(self) -> str:
        return self.selected

class UIButton(UIComponent):
    """Button-Komponente"""

    def __init__(self, scene_ref, text: str, callback: Callable,
                 css_class: str = ""):
        super().__init__(scene_ref)
        self.text = text
        self.callback = callback
        self.css_class = css_class
        self.button_widget = None

    def render(self):
        """Rendert den Button"""
        #self.scene.append_to_caption("<div class='bnt-group'>")
        self.button_widget = button(text=self.text, bind=self._on_click, canvas=self.scene) #TODO
        #self.scene.append_to_caption(f"</div>")
        return self

    def _on_click(self, evt):
        """Interner Callback für Button-Klicks"""
        if self.enabled and self.callback:
            self.callback()

    def set_text(self, text: str):
        """Ändert den Button-Text"""
        self.text = text
        if self.button_widget:
            self.button_widget.text = text


# ----------------------------------------
# UI Section (Container für Components)
# ----------------------------------------

class UISection:
    """Container für eine Gruppe von UI-Komponente"""

    def __init__(self, scene_ref, title: str, icon: str = ""):
        self.scene = scene_ref
        self.title = title
        self.icon = icon
        self.components: list[UIComponent] = []

    def begin(self):
        """Startet die Section"""
        self.scene.append_to_caption(f"<div class='ui-section'><h3>{self.icon} {self.title}</h3>")   #TODO ui-section
        return self

    def end(self):
        """Beendet die Section"""
        self.scene.append_to_caption("</div>")
        return self

    def add_component(self, component: UIComponent):
        """Fügt eine Komponente hinzu"""
        self.components.append(component)
        return self

    def add_slider(self, config: SliderConfig) -> UISlider:
        """Fügt einen Slider hinzu"""
        slider = UISlider(self.scene, config)
        slider.render()
        self.components.append(slider)
        return slider
        #return self.add_component(slider)

    def add_checkbox(self, label: str, checked: bool = True,
                     callback: Optional[Callable] = None) -> UICheckbox:
        """Fügt einen Check"""
        cb = UICheckbox(self.scene, label, checked, callback)
        cb.render()
        self.components.append(cb)
        return cb
        #return self.add_component(cb)

    def add_menu(self, label: str, choices: list,
                 selected: Optional[str] = None,
                 callback: Optional[Callable] = None) -> UIMenu:
        """Fügt ein Menü hinzu"""
        m = UIMenu(self.scene, label, choices, selected, callback)
        m.render()
        self.components.append(m)
        return m
        #return self.add_component(m)

    def add_button(self, text: str, callback: Callable) -> UIButton:
        """Fügt einen Button hinzu"""
        btn = UIButton(self.scene, text, callback)
        btn.render()
        self.components.append(btn)
        return btn
        #return self.add_component(btn)

    def add_info_box(self, text: str):
        """Fügt eine Info-Box hinzu"""
        self.scene.append_to_caption(f"<div class='info_box'>{text}</div>")


# ----------------------------------------
# Main UI Controller
# ----------------------------------------

class SimulationUIController:
    """Hauptcontroller für die gesamte UI"""

    def __init__(self, scene_ref, simulation_controller):
        self.scene = scene_ref
        self.sim_controller = simulation_controller
        self.sections: Dict[str, UISection] = {}

        # UI-Komponenten Referenten
        self.run_button: Optional[UIButton] = None
        self.mesh_speed_slider: Optional[UISlider] = None
        self.cube_speed_slider: Optional[UISlider] = None

        self._initialize_ui()

    def _initialize_ui(self):
        """Initialisiert die komplette UI"""

        # Styles injizieren
        self.scene.append_to_caption(UIStyleManager.get_slidebar_style())

        # Sidebar Container öffnen
        #self.scene.append_to_caption("<div class='control-panel'>")    # TODO

        # Sections erstellen
        self._create_control_section()
        self._create_material_section()
        self._create_parameter_section()
        self._create_visualization_section()

        # Sidebar schließen
        #self.scene.append_to_caption("</div>")

    def _create_control_section(self):
        """Erstellt die Steuerungs-Sektion"""
        section = UISection(self.scene, "Simulation", "⚙️")
        section.begin()

        #self.scene.append_to_caption("<div style='display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 15px 0;'>")
        #self.scene.append_to_caption("<div class='btn-group'>")

        #Start/Pause Button
        self.run_button = section.add_button(
            "▶︎ Start",
            callback=self.sim_controller.toggle_run
        )

        #self.scene.append_to_caption("<br>")
        self.scene.append_to_caption("&nbsp;&nbsp;")    # TODO

        # Reset Button
        section.add_button(
            "↺ Reset",
            callback=self.sim_controller.reset
        )
        self.scene.append_to_caption("<br><br>")

        section.end()
        self.sections['control'] = section


    def _create_material_section(self):
        """Erstellt die Material-Auswahl Sektion"""
        section = UISection(self.scene, "Material", "🧱")
        section.begin()

        # Material Mesh
        section.add_menu(
            label="Mesh Material",
            choices=['Holz', 'Stahl', '[Aluminium]', '[Gummi]'],
            callback=lambda m: self.sim_controller.change_mesh_material(m)
        )

        # Material Cube
        section.add_menu(
            label="Cube Material",
            choices=['Holz', 'Stahl', '[Aluminium]', '[Gummi]'],
            callback=lambda m: self.sim_controller.change_cube_material(m)
        )

        section.end()
        self.sections['material'] = section


    def _create_parameter_section(self):
        """Erstellt die Parameter-Sektion"""
        section = UISection(self.scene, "Parameter", "📊")
        section.begin()

        # Damping Slider

        # Mesh Speed Slider
        #self.scene.append_to_caption("<div style='margin: 10px 0;'>")
        mesh_speed_label = wtext(text="Geschw. Mesh: ")
        mesh_speed_value = wtext(text=f"{1.0:.1f}x")
        self.scene.append_to_caption("<br>")

        self.mesh_speed_slider = slider( #section.add_slider(SliderConfig(
            #label="Geschw. Mesh",
            min=0.0,
            max=2.0,
            value=1.0,
            step=0.01,
            #unit="x",
            #value_id="mesh-speed-val",
            #callback=self.sim_controller.set_mesh_speed
            bind=lambda s: self._update_mesh_speed(s, mesh_speed_value)
        )
        #self.scene.append_to_caption("</div>")

        # Cube Speed Slider
        #self.scene.append_to_caption("<div style='margin: 10px 0;'>")
        cube_speed_label = wtext(text="<br>Geschw. Cube: ")
        cube_speed_value = wtext(text=f"{1.0:.1f}x")
        self.scene.append_to_caption("<br>")

        self.cube_speed_slider = slider( #section.add_slider(SliderConfig(
            #label="Geschw. Cube",
            min=0.0,
            max=2.0,
            value=1.0,
            step=0.01,
            #unit="x",
            #value_id="cube-speed-val",
            #callback=self.sim_controller.set_cube_speed
            bind=lambda s: self._update_cube_speed(s, cube_speed_value)
        )
        #self.scene.append_to_caption("</div>")

        section.end()
        self.sections['parameter'] = section

    def _create_visualization_section(self):
        """Erstellt die Visualisierungs-Sektion"""
        section = UISection(self.scene, "Visualisierung", "🎨")
        section.begin()

        # Checkboxen
        section.add_checkbox(
            "Freiheitsgrade",
            checked=True,
            callback=self.sim_controller.toggle_arrows
        )

        section.add_checkbox(
            "Kontaktpunkte",
            checked=True,
            callback=self.sim_controller.toggle_contacts
        )

        section.add_checkbox(
            "Bewegungsspuren",
            checked=True,
            callback=self.sim_controller.toggle_trail
        )

        # Info-Box
        section.add_info_box(
            "<b>Tipp:</b> Rechtsklick + Ziehen zum Drehen der Kamera <br>"
            "    zum Zoomen -> Mausrad verwenden"
        )

        section.end()
        self.sections['visualisation'] = section

    def update_run_button(self, is_running: bool):
        """Aktualisiert dem Start/Pause Button"""
        if self.run_button:
            text="❚❚ Pause" if is_running else "▶ Start"
            self.run_button.set_text(text)

    def lock_speed_sliders(self, locked: bool):
        """Sperrt/Entsperrt die Geschwindigkeits-Slider"""
        #if self.mesh_speed_slider:
        #    self.mesh_speed_slider.set_enabled(not locked)
        #if self.cube_speed_slider:
        #    self.cube_speed_slider.set_enabled(not locked)
        pass

    def _update_mesh_speed(self, s, value_weight):
        """Update Mesh Speed"""
        value_weight.text = f"{s.value:.1f}"
        if hasattr(self.sim_controller, "set_mesh_speed"):
            self.sim_controller.set_mesh_speed(s.value)

    def _update_cube_speed(self, s, value_weight):
        """Update Mesh Speed"""
        value_weight.text = f"{s.value:.1f}"
        if hasattr(self.sim_controller, "set_cube_speed"):
            self.sim_controller.set_cube_speed(s.value)

# ----------------------------------------
# Integration Helper
# ----------------------------------------

class VPythonSceneBuilder:
    """Helper zum Erstellen der YPython-Szene mit UI"""

    @staticmethod
    def create_scene(title: str, width: int = 1100, height: int = 950):
        """Erstellt eine VPython-Szene optimiert mit UI"""
        scene = canvas(
            title=f"<h2 style='margin:0; padding:10px 0; white-space: normal;'>{title}</h2>",
            width=width,
            height=height,
            align="left",
            background=color.white,
        )
        scene.range = 0.25,
        #scene.caption = ""
        return scene

# ----------------------------------------
# Beispiel-Verwendung (für tests)
# ----------------------------------------
"""
Implementierung in programmCOntroller.py integrieren
"""
"""
if __name__ == "__main__":

    # Mock Simulation Controller
    class MockSimController:
        def __init__(self):
            self.running = False

        def toggle_run(self):
            self.running = not self.running
            print(f"Running: {self.running}")

        def reset(self):
            print("Reset called")

        def change_mesh_material(self, material):
            print(f"Mesh Material: {material}")

        def change_cube_material(self, material):
            print(f"Cube Material: {material}")

        def set_mesh_speed_value(self, value):
            print(f"Set mesh speed value: {value}")

        def set_cube_speed_value(self, value):
            print(f"Set cube speed value: {value}")

        def toggle_arrows(self, checked: bool):
            print(f"Toggle arrows checked: {checked}")

        def toggle_contacts(self, checked: bool):
            print(f"Toggle contacts checked: {checked}")

        def toggle_trail(self, checked: bool):
            print(f"Toggle trail checked: {checked}")

    # Szene erstellen
    scene = VPythonSceneBuilder.create_scene("Test UI")

    # UI Controller erstellen
    mock_controller = MockSimController()
    ui = SimulationUIController(scene, mock_controller)

    print("UI erfolgreich erstellt")

    while True:
        rate(60)

"""







