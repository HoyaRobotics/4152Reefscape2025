// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectivetracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

// I need to have a directed graph of poses
// then run dijkstras algorithm given a target pose
// which should return a list of the shortets poses on the way
// generate a supplier which will move through each pose, going onto the next
// when within tolerance except for the last pose
// + fuse driver inputs the whole way, and have some easy way of disabling it??
// have the subsystem / manager periodically check if it has a coral objective and
// the intake has an objective and this operator feature is enabled (also through the
// web interface?) and then run the command until its disabled or places the coral
public class CoralPlanner extends SubsystemBase {

    // undirected, weighted, cyclic graph
    private Set<Node> graph = Reef.getAllianceReefList().stream()
            .map(pose -> Reef.offsetReefPose(pose, Side.CENTER))
            .flatMap(pose -> Stream.of(
                    new Node(pose.transformBy(new Transform2d(1.0, 0.5, Rotation2d.kZero))),
                    new Node(pose.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero))),
                    new Node(pose.transformBy(new Transform2d(1.0, -0.5, Rotation2d.kZero)))))
            .collect(Collectors.toSet());

    public record CoralObjective(int faceIndex, Side branchSide, SuperStructurePose superStructurePose) {}

    private Optional<CoralObjective> currentObjective = Optional.empty();

    public class Node {
        private Pose2d pose;
        private Double distance = Double.MAX_VALUE;
        private Node prev;
        private Map<Node, Double> adjacentNodes = new HashMap<>();

        public Node(Pose2d pose) {
            this.pose = pose;
        }

        public Map<Node, Double> getAdjacentNodes() {
            return adjacentNodes;
        }

        public void setDistance(Double distance) {
            this.distance = distance;
        }

        public Double getDistance() {
            return this.distance;
        }

        public void setPrev(Node prev) {
            this.prev = prev;
        }

        public Pose2d getPose() {
            return pose;
        }

        public void addEdge(Node node) {
            this.adjacentNodes.put(node, node.getPose().getTranslation().getDistance(this.pose.getTranslation()));
        }
    }

    private final StringSubscriber networkedObjective = NetworkTableInstance.getDefault()
            .getStringTopic("SmartDashboard/CoralObjective")
            .subscribe("default");

    /** Creates a new ObjectiveTracker. */
    public CoralPlanner() {
        // add edges to graph
        Node lastNode = null;
        Node firstNode = null;
        for (Node node : graph) {
            if (lastNode != null) {
                node.addEdge(lastNode);
                lastNode.addEdge(node);
            } else {
                firstNode = node;
            }
            lastNode = node;
        }

        if (firstNode != null && lastNode != null) {
            firstNode.addEdge(lastNode);
            lastNode.addEdge(firstNode);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        currentObjective = Optional.of(getOpObjective());
    }

    private CoralObjective getOpObjective() {
        // error handling!!
        String coralObjective = networkedObjective.get();
        Side branchSide = coralObjective.charAt(3) == 'L' ? Side.LEFT : Side.RIGHT;
        int faceIndex = (coralObjective.charAt(0) - '0' + 2) % 5;

        SuperStructurePose superStructurePose = SuperStructurePose.BASE;
        switch (coralObjective.charAt(2)) {
            case 'T' -> superStructurePose = SuperStructurePose.L4;
            case 'M' -> superStructurePose = SuperStructurePose.L3;
            case 'B' -> superStructurePose = SuperStructurePose.L2;
        }
        ;

        return new CoralObjective(faceIndex, branchSide, superStructurePose);
    }

    public List<Pose2d> dijkstraPath(Pose2d drivePose) {
        if (currentObjective.isEmpty()) {
            return List.of();
        }

        Node sourceNode = graph.stream()
                .min(Comparator.comparing(
                        node -> ((Node) node).getPose().getTranslation().getDistance(drivePose.getTranslation())))
                .get();

        // final node will be the target face, extended by like 1 meter
        //
        //            L     T     R
        //               /  |  \
        //              /   |   \
        //             *    *    *
        //            --------------
        //           /              \
        //          /                \
        //         /                  \
        //         \                  /
        //          \                /
        //           \              /
        //            --------------

        int finalFace = currentObjective.get().faceIndex;
        Side finalSide = currentObjective.get().branchSide;

        Node targetNode = graph.stream()
                .filter(node -> node.getPose().equals(
                    Reef.getAllianceReefBranch(finalFace, Side.CENTER)
                        .transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero))))
                .findFirst()
                .get();

        sourceNode.setDistance(0.0);

        Queue<Node> toVisit = new PriorityQueue<>(Collections.singleton(sourceNode));

        // keep track of path
        while (!toVisit.isEmpty()) {
            Node currentNode = toVisit.poll();

            for (var entry : currentNode.getAdjacentNodes().entrySet()) {
                Node neighbour = entry.getKey();
                Double currentToNeighbour = entry.getValue();

                Double newDistance = currentToNeighbour + currentNode.getDistance();

                if (newDistance < neighbour.getDistance()) {
                    neighbour.setDistance(newDistance);
                    neighbour.setPrev(currentNode);
                    toVisit.add(neighbour);
                }
            }
        }

        Node currentNode = targetNode;

        List<Pose2d> finalPath = new ArrayList<>();
        finalPath.add(Reef.getAllianceReefBranch(finalFace, finalSide));

        while (currentNode != sourceNode) {
            finalPath.add(currentNode.getPose());
            currentNode = currentNode.prev;
        }
        finalPath.add(sourceNode.getPose());

        Collections.reverse(finalPath);
        return finalPath;
    }
}
