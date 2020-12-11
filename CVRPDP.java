package class118133.trandinhhung;

import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class CVRPDP {
    static {
        System.loadLibrary("jniortools");
    }
    MPSolver solver ;
    CVRPDP_data cvrpdp_data;
    MPVariable[][][] x;
    MPVariable[][] y,z,t;

    public CVRPDP(String filename) {
        this.cvrpdp_data = new CVRPDP_data(filename);
        this.x = new MPVariable[cvrpdp_data.K+1][cvrpdp_data.distanceMatrix.length+2][cvrpdp_data.distanceMatrix.length+2];
        this.y = new MPVariable[cvrpdp_data.K+1][cvrpdp_data.distanceMatrix.length+2];
        this.z = new MPVariable[cvrpdp_data.K+1][2*cvrpdp_data.N+2];
        this.t = new MPVariable[cvrpdp_data.K+1][2*cvrpdp_data.N+2];
        this.solver = MPSolver.createSolver("CRVPDP solver","CBC");
    }

    private void solve() {
        System.out.println("Solver is starting...");
        int N = cvrpdp_data.N;                  //  Number of customer
        int K = cvrpdp_data.K;                  //  Number of bus
        int[] q = cvrpdp_data.q;                //  Capacities
        int[][] d = cvrpdp_data.distanceMatrix; //  Distance matrix
        int M = 3*N;                            //  A big enough number

        //  x[k][i][j] = 1: Bus k travels from i to j
        for (int i = 0; i <= 2*N+1; i++) {
            for (int j = 0; j <= 2*N+1; j++) {
                for (int k = 1; k <= K; k++) {
                    this.x[k][i][j] = this.solver.makeIntVar(0, 1, "X[" + k + "," + i + "," + j + "]");
                }
            }
        }

        //  y[k][i]: Number of customer on bus k after leaving node i
        for (int i = 0; i <= 2*N+1; i++) {
            for (int k = 1; k <= K; k++) {
                this.y[k][i] = this.solver.makeIntVar(0, N, "Y[" + k + "," + i + "]");
            }
        }

        //  z[k][i] = r: Bus k visits node i on its r-th move
        //  t[k][i] = 1: Bus k visits node i
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= 2*N+1; i++) {
                this.z[k][i] = this.solver.makeIntVar(0, 2*N+1, "Z[" + k + "," + i + "]");
                this.t[k][i] = this.solver.makeIntVar(0, 1, "T[" + k + "," + i + "]");
            }
        }

        //  Create depot at node 2N+1
        for (int i = 0; i <= N; i++) {
            d[i][2*N+1] = M;
            d[2*N+1][i] = M;
        }
        for (int i = N+1; i <= 2*N; i++) {
            d[i][2*N+1] = 0;
            d[2*N+1][i] = M;
        }

        //  ADD CONSTRAINTS (12)
        //  Each node is visited exactly once by exactly one bus
        for (int j = 1; j <= 2*N; j++) {
            MPConstraint c = solver.makeConstraint(1, 1);
            for (int k = 1; k <= K; k++) {
                for (int i = 0; i <= 2*N; i++) {
                    if (i != j) {
                        c.setCoefficient(this.x[k][i][j], 1);
                    }
                }
            }
            c = solver.makeConstraint(1, 1);
            for (int k = 1; k <= K; k++) {
                for (int i = 1; i <= 2*N+1; i++) {
                    if (i != j) {
                        c.setCoefficient(this.x[k][j][i], 1);
                    }
                }
            }
        }

        //  If bus k visits node i then it must leave i
        for (int i = 1; i <= 2*N; i++) {
            for (int k = 1; k <= K; k++) {
                MPConstraint c = solver.makeConstraint(0, 0);
                for (int j = 0; j <= 2*N; j++) {
                    if (j != i) {
                        c.setCoefficient(this.x[k][j][i], 1);
                    }
                }
                for (int j = 1; j <= 2*N+1; j++) {
                    if (j != i) {
                        c.setCoefficient(this.x[k][i][j], -1);
                    }
                }
            }
        }

        //  If bus k departs then it terminates at node 2N+1
        for (int k = 1; k <= K; k++) {
            MPConstraint c = solver.makeConstraint(0, 1);
            for (int i = 0; i <= 2*N; i++) {
                c.setCoefficient(x[k][i][2*N+1], 1);
            }
        }

        //  Each bus departs from node 0 (or not travel)
        for (int k = 1; k <= K; k++) {
            MPConstraint c = solver.makeConstraint(0, 0);
            c.setCoefficient(this.z[k][0], 1);
        }

        //  If bus k visits i then t[k][i] = 1
        for (int k = 1; k <= K; k++) {
            for (int i = 1; i <= 2*N; i++) {
                MPConstraint c = solver.makeConstraint(0, 0);
                for (int j = 1; j <= 2*N+1; j++) {
                    if (j != i) {
                        c.setCoefficient(this.x[k][i][j], 1);
                    }
                }
                c.setCoefficient(this.t[k][i], -1);
            }
        }

        //  If bus k visits i then it must visit i+N (i=1,2,3,...,N)
        for (int k = 1; k <= K; k++) {
            for (int i = 1; i <= N; i++) {
                MPConstraint c = solver.makeConstraint(0, 0);
                c.setCoefficient(t[k][i],1);
                c.setCoefficient(t[k][i+N],-1);
            }
        }

        //  Bus k must visit node i before it visits node i+N (i=1,2,3,...,N)
        for (int k = 1; k <= K; k++) {
            for (int i = 1; i <= N; i++) {
                MPConstraint c = solver.makeConstraint(0, 2*N);
                c.setCoefficient(z[k][i],-1);
                c.setCoefficient(z[k][i+N],1);
                c.setCoefficient(t[k][i], -1);
            }
        }

        //  Bus k travels from i to j then z[k][i] + 1 = z[k][j]
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= 2*N; i++) {
                for (int j = 1; j <= 2*N+1; j++) {
                    if (j != i) {
                        MPConstraint c = solver.makeConstraint(-1-M, M);
                        c.setCoefficient(this.x[k][i][j], -M);
                        c.setCoefficient(this.z[k][i], 1);
                        c.setCoefficient(this.z[k][j], -1);

                        c = solver.makeConstraint(1-M, M);
                        c.setCoefficient(this.x[k][i][j], -M);
                        c.setCoefficient(this.z[k][i], -1);
                        c.setCoefficient(this.z[k][j], 1);
                    }
                }
            }
        }

        //  Each bus is empty when it departs
        for (int k = 1; k <= K; k++) {
            MPConstraint c = solver.makeConstraint(0, 0);
            c.setCoefficient(this.y[k][0], 1);
        }

        //  If bus k visit node j to pick up customer then y[k,i] + 1 = y[k,j] (j=1,2,3,...,N)
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= 2*N; i++) {
                for (int j = 1; j <= N; j++) {
                    if (j != i) {
                        MPConstraint c = solver.makeConstraint(1-M, M);
                        c.setCoefficient(this.x[k][i][j], -M);
                        c.setCoefficient(this.y[k][j],1);
                        c.setCoefficient(this.y[k][i],-1);

                        c = solver.makeConstraint(-1-M, M);
                        c.setCoefficient(this.x[k][i][j], -M);
                        c.setCoefficient(this.y[k][j],-1);
                        c.setCoefficient(this.y[k][i],1);
                    }
                }
            }
        }

        //  If bus k visit node j to drop off customer then y[k,i] - 1 = y[k,j] (j=N+1,N+2,N+3,...,2N)
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= 2*N; i++) {
                for (int j = N+1; j <= 2*N; j++) {
                    if (j != i) {
                        MPConstraint c = solver.makeConstraint(-1-M, M);
                        c.setCoefficient(this.x[k][i][j], -M);
                        c.setCoefficient(this.y[k][j],1);
                        c.setCoefficient(this.y[k][i],-1);

                        c = solver.makeConstraint(1-M, M);
                        c.setCoefficient(this.x[k][i][j], -M);
                        c.setCoefficient(this.y[k][j],-1);
                        c.setCoefficient(this.y[k][i],1);
                    }
                }
            }
        }

        //  Bus k must not carry more than q[k] customer
        for (int k = 1; k <= K; k++) {
            for (int i = 1; i <= 2*N; i++) {
                MPConstraint c = solver.makeConstraint(0, q[k]);
                c.setCoefficient(this.y[k][i], 1);
            }
        }

        //  OBJECTIVE FUNCTION
        MPObjective obj = solver.objective();
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= 2*N+1; i++) {
                for (int j = 0; j <= 2*N+1; j++) {
                    obj.setCoefficient(this.x[k][i][j], d[i][j]);
                }
            }
        }
        obj.setMinimization();

        MPSolver.ResultStatus res_stat = solver.solve();
        if (res_stat != MPSolver.ResultStatus.OPTIMAL) {
            System.err.println("The problem does not have optimal solution");
            return;
        } else {
            System.out.println("Solved");
        }
        System.out.println("Objective Value: " + solver.objective().value());
        for (int k = 1; k <= K; k++) {
            System.out.print("Bus " + k + ": 0");
            for (int i = 1; i <= 2*N; i++) {
                for (int j = 1; j <= 2*N; j++) {
                    if (t[k][j].solutionValue()==1 && z[k][j].solutionValue()==i)
                        System.out.print(" -> " + j);
                }
            }
            System.out.println();
        }
        System.out.println();
    }

    public static void main(String[] args) {
        CVRPDP cvrpdp = new CVRPDP("./src/class118133/trandinhhung/data1.txt");
        long start = System.currentTimeMillis();
        cvrpdp.solve();
        long end = System.currentTimeMillis();
        NumberFormat formatter = new DecimalFormat("#0.000");
        System.out.print("Execution time is " + formatter.format((end - start) / 1000d) + " seconds");
    }
}
