GCS_ANONYMOUS_ACCESS=true \ 
XLA_PYTHON_CLIENT_MEM_FRACTION=0.95 \
python scripts/train.py fran_forcevla_lora \
    --exp-name=fran_forcevla_experiment_1 \
    --batch_size 16 \
    --save_interval 2000 \
    --keep_period 10000 \
    --checkpoint_base_dir /mnt/Data/ForceVLA/checkpoints \
    --overwrite
