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

def do_data(args):
    if args.method == 'prbs':
        print("\n>>> STEP: DATA ACQUISITION (PRBS) <<<")
        run_cmd(f"python scripts/sysid_prbs_logger.py --port {args.port} --duration {args.duration} --step_time 2.0 --min_duty {args.min_duty} --max_duty {args.max_duty}")
    elif args.method == 'stair':
        print("\n>>> STEP: DATA ACQUISITION (STAIR-STEP) <<<")
        run_cmd(f"python scripts/sysid_stair_logger.py --port {args.port} --step_time 4.0")
    else:
        print("\n>>> STEP: DATA ACQUISITION (STEP) <<<")
        run_cmd(f"python scripts/sysid_logger.py --port {args.port} --duty {args.duty} --duration {args.duration}")

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
    parser.add_argument('--method', type=str, choices=['step', 'prbs', 'stair'], default='step', help='Data acquisition method')
    parser.add_argument('--port', type=str, default='COM16', help='Serial port for hardware tests')
    parser.add_argument('--duty', type=float, default=0.6, help='Duty cycle for open-loop sysid (STEP method)')
    parser.add_argument('--min_duty', type=float, default=0.1, help='Min duty for PRBS method')
    parser.add_argument('--max_duty', type=float, default=0.9, help='Max duty for PRBS method')
    parser.add_argument('--rpm', type=float, default=1000.0, help='Target RPM for closed-loop test')
    parser.add_argument('--duration', type=float, default=50.0, help='Duration of data acquisition in seconds')
    
    args = parser.parse_args()
    
    if args.action == 'data':
        do_data(args)
        
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
        do_data(args)
        do_ident_pidtune()
        do_ident_pso()
        do_run_pidtune(args.rpm)
        do_run_pso(args.rpm)
