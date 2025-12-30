package dev.zedboy.greatness.math;

public class mat {
    double[][] data;

    public mat(int rows, int cols) {
        if (rows < 1 || cols < 1) {
            rows = 1;
            cols = 1;
        }

        this.data = new double[rows][cols];
    }

    public mat(double[][] data) {
        if (data.length == 0 || data[0].length == 0) {
            this.data = new double[1][1];
            return;
        }

        this.data = data;
    }

    public int rows() {
        return this.data.length;
    }

    public int cols() {
        return this.data[0].length;
    }

    public static mat identity(int size) {
        mat mat = new mat(size, size);
        for (int i = 0; i < size; i++) mat.data[i][i] = 1;

        return mat;
    }
}
