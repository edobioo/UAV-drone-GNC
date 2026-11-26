clear all, clc, close all

# Timing
fs = 200;
ts = 1 / fs;
t_end = 10000;
n_samples = t_end / ts;
t_vec = 0 : ts : t_end;

t_vec(end)
