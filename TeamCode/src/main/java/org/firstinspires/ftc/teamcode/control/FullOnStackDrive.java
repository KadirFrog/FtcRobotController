package org.firstinspires.ftc.teamcode.control;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Position;

import java.util.ArrayList;

public class FullOnStackDrive {
    class ExtendedPosition {

        public Position position;
        public final float width, length, height;

        public ExtendedPosition(Position starting_position, float width, float length, float height) {
            this.position = starting_position;
            this.width = width;
            this.length = length;
            this.height = height;

        }
        public void set_position(Position new_position) {
            this.position = new_position;
        }

        public boolean does_collide(@NonNull ExtendedPosition colliding_object) {
            return (this.position.x + this.width / 2 > colliding_object.position.x - colliding_object.width / 2 &&
                    this.position.x - this.width / 2 > colliding_object.position.x + colliding_object.width / 2)
                    || (this.position.y + this.length / 2 > colliding_object.position.y - colliding_object.length / 2
                    && this.position.y - this.length / 2 > colliding_object.position.y + colliding_object.length);
        }

    }

    class Drive {
        private ArrayList<ExtendedPosition> Obstacles;
        public void set_obstacles(ArrayList<ExtendedPosition> obstacles) {
            this.Obstacles = obstacles;
        }
        public void add_obstacle(ExtendedPosition obstacle) {
            this.Obstacles.add(obstacle);
        }
        private final int correctness_in_portion;
        private final float max_total_deviation;
        private final ExtendedPosition robot_simulated_position;
        public Drive(int correctness_in_portion, ExtendedPosition robot_simulated_position, float max_total_deviation) {
            this.correctness_in_portion = correctness_in_portion;
            this.robot_simulated_position = robot_simulated_position;
            this.max_total_deviation = max_total_deviation;
        }
        public ExtendedPosition check_all_obstacle_collisions() {
            ExtendedPosition obstacle;
            for (int count = 0; count < this.Obstacles.size(); count++) {
                obstacle = this.Obstacles.get(count);
                if (this.robot_simulated_position.does_collide(obstacle)) {
                    return obstacle;
                }
            }
            return null;
        }
        public ArrayList<Position> get_loc(Position wished_position) {
            double positional_difference_x = abs(this.robot_simulated_position.position.x - wished_position.x);
            double positional_difference_y = abs(this.robot_simulated_position.position.y - wished_position.y);

            ArrayList<Position> dodging_positions = new ArrayList<>();

            boolean colliding = false;
            while (!colliding) {
                ExtendedPosition collided_obstacle = check_all_obstacle_collisions();
                while (collided_obstacle != null) {
                    if (this.robot_simulated_position.position.x > positional_difference_x - this.max_total_deviation
                            && this.robot_simulated_position.position.x < positional_difference_x + this.max_total_deviation) {

                        double save_var_x = this.robot_simulated_position.position.x;
                        double save_var_y = this.robot_simulated_position.position.y;
                        dodging_positions.add(new Position(save_var_x, save_var_y, this.robot_simulated_position.position.rotation));

                        this.robot_simulated_position.position.y += positional_difference_x / this.correctness_in_portion; // Adjust this value accordingly
                    }
                    collided_obstacle = check_all_obstacle_collisions();
                }

                this.robot_simulated_position.position.x += 1;

                colliding = this.robot_simulated_position.position.equals(wished_position);
            }


        return dodging_positions;
        }

}}
