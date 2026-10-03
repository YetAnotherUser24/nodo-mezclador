import argparse
import subprocess
import sys
import os

def run_cmd(cmd):
    print("==================================================")
    print(f"RUNNING: {cmd}")
    print("==================================================")
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        print(f"\n[ERROR] Command failed with exit code {result.returncode}: {cmd}")
        sys.exit(1)

def get_gains(filepath):
    if not os.path.exists(filepath):
        print(f"\n[ERROR] {filepath} not found. Run the 'ident' step first!")
        sys.exit(1)
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('--kp'):
                return line.strip()
    print(f"\n[ERROR] Could not find gains inside {filepath}")
    sys.exit(1)

def do_data(port, duty):
    print("\n>>> STEP: DATA ACQUISITION <<<")
    run_cmd(f"python scripts/sysid_logger.py --port {port} --duty {duty} --duration 5.0")

def do_ident_pso():
    print("\n>>> STEP: IDENT & TUNE (PSO) <<<")
    run_cmd("matlab -batch \"cd scripts; run('advanced_thesis_tuning.m')\"")

def do_ident_pidtune():
    print("\n>>> STEP: IDENT & TUNE (PIDTUNE) <<<")
    run_cmd("matlab -batch \"cd scripts; run('sysid_and_tune.m')\"")

def do_run_pso(rpm):
    print("\n>>> STEP: RUN CLOSED-LOOP (PSO) <<<")
    gains = get_gains("scripts/results/models/pso_gains.txt")
    run_cmd(f"python scripts/test_gains.py {gains} --rpm {rpm} --output scripts/data/closed_loop_pso.csv")
    run_cmd("matlab -batch \"cd scripts; csv_file='data/closed_loop_pso.csv'; mat_file='results/models/sysid_results.mat'; out_img='results/figures/pso/closed_loop_validation.png'; run('plot_validation.m')\"")

def do_run_pidtune(rpm):
    print("\n>>> STEP: RUN CLOSED-LOOP (PIDTUNE) <<<")
    gains = get_gains("scripts/results/models/pidtune_gains.txt")
    run_cmd(f"python scripts/test_gains.py {gains} --rpm {rpm} --output scripts/data/closed_loop_pidtune.csv")
    run_cmd("matlab -batch \"cd scripts; csv_file='data/closed_loop_pidtune.csv'; mat_file='results/models/sysid_results_pidtune.mat'; out_img='results/figures/pidtune/closed_loop_validation.png'; run('plot_validation.m')\"")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="T-200 Controller Pipeline Manager")
    parser.add_argument('action', choices=[
        'full', 
        'data', 
        'apply', 
        'apply-pso', 
        'apply-pidtune', 
        'ident', 
        'ident-pso', 
        'ident-pidtune', 
        'run',
        'run-pso', 
        'run-pidtune'
    ], help="The action to perform in the pipeline.")
    
    parser.add_argument('--port', type=str, default='COM16', help='Serial port for hardware tests')
    parser.add_argument('--duty', type=float, default=0.6, help='Duty cycle for open-loop sysid')
    parser.add_argument('--rpm', type=float, default=1000.0, help='Target RPM for closed-loop test')
    
    args = parser.parse_args()
    
    if args.action == 'data':
        do_data(args.port, args.duty)
        
    elif args.action == 'ident':
        do_ident_pidtune()
        do_ident_pso()
        
    elif args.action == 'ident-pso':
        do_ident_pso()
        
    elif args.action == 'ident-pidtune':
        do_ident_pidtune()
        
    elif args.action == 'run':
        do_run_pidtune(args.rpm)
        do_run_pso(args.rpm)
        
    elif args.action == 'run-pso':
        do_run_pso(args.rpm)
        
    elif args.action == 'run-pidtune':
        do_run_pidtune(args.rpm)
        
    elif args.action == 'apply':
        do_ident_pidtune()
        do_ident_pso()
        do_run_pidtune(args.rpm)
        do_run_pso(args.rpm)
        
    elif args.action == 'apply-pso':
        do_ident_pso()
        do_run_pso(args.rpm)
        
    elif args.action == 'apply-pidtune':
        do_ident_pidtune()
        do_run_pidtune(args.rpm)
        
    elif args.action == 'full':
        do_data(args.port, args.duty)
        do_ident_pidtune()
        do_ident_pso()
        do_run_pidtune(args.rpm)
        do_run_pso(args.rpm)
