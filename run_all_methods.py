import os
import shutil
import subprocess

def run_method(method, name):
    print(f"\n====================================")
    print(f"Running pipeline for method: {method}")
    print(f"====================================")
    
    # Run the pipeline
    cmd = f"python pipeline.py full --method {method}"
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        print(f"Pipeline failed for {method}")
        return False
        
    # Destination directory for artifacts
    artifact_dir = r"C:\Users\Sam\.gemini\antigravity-ide\brain\3678bb67-10bd-44a0-8497-e367b399917d"
    
    # Files to copy
    # sysid_raw_data.png (from PIDTUNE since it plots the raw data)
    src_raw = r"scripts\results\figures\pidtune\sysid_raw_data.png"
    if os.path.exists(src_raw):
        shutil.copy(src_raw, os.path.join(artifact_dir, f"{name}_raw_data.png"))
        
    # Model Fit (from PSO)
    src_fit = r"scripts\results\figures\pso\sysid_model_fit.png"
    if os.path.exists(src_fit):
        shutil.copy(src_fit, os.path.join(artifact_dir, f"{name}_model_fit.png"))
        
    # Closed Loop Validation (from PSO)
    src_val = r"scripts\results\figures\pso\closed_loop_validation.png"
    if os.path.exists(src_val):
        shutil.copy(src_val, os.path.join(artifact_dir, f"{name}_pso_validation.png"))

    # Closed Loop Validation (from PIDTUNE)
    src_val_pid = r"scripts\results\figures\pidtune\closed_loop_validation.png"
    if os.path.exists(src_val_pid):
        shutil.copy(src_val_pid, os.path.join(artifact_dir, f"{name}_pidtune_validation.png"))
        
    print(f"Successfully copied artifacts for {method}.")
    return True

if __name__ == "__main__":
    run_method("step", "step")
    run_method("prbs", "prbs")
    run_method("stair", "stair")
