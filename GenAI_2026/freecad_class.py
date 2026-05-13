import os
import json
import subprocess

class FreeCADClass:

    def __init__(self):
        self.output_dir = "outputs"
        os.makedirs(self.output_dir, exist_ok=True)


    def llm_generate_steps(self, prompt):
        """
        Function that  mocks an LLM and creates the desired shape.
        """
        # get the prompt in lowercase
        prompt = prompt.lower()

        # list of the actions to be taken by freecad
        self.action = []

        # check for each shape in the prompt
        if "cube" in prompt:
            print("here")
            self.action.append({"action": "box", "l": 10, "w": 10, "h": 10})
        
        if "hole" in prompt:
            self.action.append({"action": "hole", "diameter": 5, "depth": 10, "x": 10, "y": 10})


    def validate_steps(self):
        # check if the steps are in list format
        if not isinstance(self.action, list):
            return False, "Steps must be a list"

        for step in self.action:
            if "action" not in step:
                return False, "Missing action field"

        return True, None


    def steps_to_freecad_script(self, filepath):
        """
        Function that creates the FreeCAD script from the action list
        """
        # start with creating the part and the document
        lines = [
            "import FreeCAD, Part",
            "doc = FreeCAD.newDocument()"
        ]

        # check for each action
        for i, step in enumerate(self.action):
            if step["action"] == "box":
                lines.append(f"box_{i} = Part.makeBox({step['l']}, {step['w']}, {step['h']})")
                lines.append(f"result = box_{i}")

            elif step["action"] == "hole":
                lines.append(f"cyl_{i} = Part.makeCylinder({step['diameter']/2}, {step['depth']})")
                lines.append(f"cyl_{i}.translate(FreeCAD.Vector({step['x']}, {step['y']}, 0))")
                lines.append(f"result = result.cut(cyl_{i})")

        lines.append("Part.show(result)")
        lines.append("doc.recompute()")
        lines.append(f"doc.saveAs(r'{filepath}')")

        return "\n".join(lines)


    def run_freecad(self, script_path, open=False):
        """
        Function that takes care of running FreeCAD.
        """
        if open:
            #! add this for Windows
            FREECAD_CMD = os.getenv(
            "FREECAD_CMD",
            r"C:\Program Files\FreeCAD 1.0\bin\freecad.exe" # if I dont find the path, then right click on the shortcut and then properties
            )
            try:
                subprocess.run([
                    FREECAD_CMD,
                    script_path
                ], check=True)
                return True, None
            except subprocess.CalledProcessError as e:
                return False, str(e)
            """
            #! Use this for Mac/Linux
            try:
                subprocess.run([
                    "freecadcmd",
                    script_path
                ], check=True)
                return True, None
            """
        else:
            print("FreeCAD Not Opening ...................")
        