package class118133.trandinhhung;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class CVRPDP_data {
    public int[][] distanceMatrix;
    public int len, N, K;
    public int[] q;
    public CVRPDP_data(String file_name) {
        File file = new File(file_name);
        try {
            Scanner sc = new Scanner(file);
            this.N = sc.nextInt();
            this.K = sc.nextInt();
            this.q = new int[K+2];
            for (int i = 1; i <= this.K; i++) {
                this.q[i] = sc.nextInt();
            }
            this.distanceMatrix = new int[2*N+2][2*N+2];
            for (int i = 0; i <= 2*this.N; i++) {
                for (int j = 0; j <= 2*this.N; j++) {
                    this.distanceMatrix[i][j] = sc.nextInt();
                }
                this.distanceMatrix[i][2*N+1] = 0;
                this.distanceMatrix[2*N+1][i] = 1000000;
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        CVRPDP_data cvrpdp_data = new CVRPDP_data("./src/class118133/trandinhhung/data.txt");
        int N = cvrpdp_data.N;
        int K = cvrpdp_data.K;
        System.out.println(N);
        System.out.println(K);
        for (int i = 0; i <= 2*N; i++) {
            for (int j = 0; j <= 2 * N; j++) {
                System.out.print(cvrpdp_data.distanceMatrix[i][j] + " ");
            }
            System.out.println();
        }
    }
}
