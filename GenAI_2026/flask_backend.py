import os
import uuid
from flask import Flask, request, jsonify, send_file
from freecad_class import FreeCADClass

app = Flask(__name__)
freecad_class = FreeCADClass()

@app.route("/steps", methods=["POST"])
def preview_steps():
    data = request.get_json()
    prompt = data.get("prompt", "")

    if not prompt:
        return jsonify({"error": "Prompt is required"}), 400

    steps = freecad_class.llm_generate_steps(prompt)
    return jsonify({"steps": steps})

# --- ROUTES ---
@app.route("/generate", methods=["POST"])
def generate():
    data = request.get_json()
    prompt = data.get("prompt", "")

    if not prompt:
        return jsonify({"error": "Prompt is required"}), 400

    # 1. LLM
    freecad_class.llm_generate_steps(prompt)

    # 2. Validate
    valid, error = freecad_class.validate_steps()
    if not valid:
        return jsonify({"error": error}), 400

    # 3. Prepare file paths
    file_id = str(uuid.uuid4())
    fcstd_path = os.path.join(freecad_class.output_dir, f"{file_id}.FCStd")
    script_path = os.path.join(freecad_class.output_dir, f"{file_id}.py")

    # 4. Generate script
    script_content = freecad_class.steps_to_freecad_script(fcstd_path)
    with open(script_path, "w") as f:
        f.write(script_content)

    # 5. Run FreeCAD
    print("Running FreeCAD...")
    success, error = freecad_class.run_freecad(script_path)
    if not success:
        return jsonify({"error": error}), 500

    # 6. Return file
    return send_file(fcstd_path, as_attachment=True)


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"})


if __name__ == "__main__":
    app.run(debug=True)
