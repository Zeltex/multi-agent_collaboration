module load python3/3.8.9
python3 setup_gcc.py install --user
pip3 install --user -e gym-cooking-fork
cd gym-cooking-fork/gym_cooking
bsub -J BD[1-45] < hpc_array.sh
