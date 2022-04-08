package jbweb;

import bwapi.TilePosition;

import java.util.*;
import java.util.stream.Stream;

public class AStar {
  public static List<TilePosition> find(TilePosition from, TilePosition to, boolean[][] walkableXY) {
      class Node {
          int cost;
          int heuristic;
          TilePosition tile;
          Node from;
      }

      Set<TilePosition> explored = new HashSet<>();

      Queue<Node> horizon = new PriorityQueue<>(Comparator.comparingInt(a -> a.heuristic));
      Node fromNode = new Node();
      fromNode.cost = 0;
      fromNode.tile = from;
      fromNode.from = null;
      horizon.add(fromNode);

      while ( ! horizon.isEmpty()) {
          final Node here = horizon.poll();
          if (here.tile.equals(to)) {
              List<TilePosition> output = new ArrayList<>();
              Node step = here;
              do {
                  output.add(step.tile);
                  step = step.from;
              } while (step != null);
              Collections.reverse(output);
              return output;
          }
          Stream.of(
              new TilePosition(here.tile.x - 1, here.tile.y),
              new TilePosition(here.tile.x + 1, here.tile.y),
              new TilePosition(here.tile.x, here.tile.y - 1),
              new TilePosition(here.tile.x, here.tile.y + 1))
                  .filter(tile -> tile.isValid(JBWEB.game) && walkableXY[tile.x][tile.y] && ! explored.contains(tile))
                  .forEach(tile -> {
                      Node neighbor = new Node();
                      neighbor.tile = tile;
                      neighbor.from = here;
                      neighbor.cost = here.cost + 1;
                      neighbor.heuristic = neighbor.cost + tile.getApproxDistance(to);
                      horizon.add(neighbor);
                      explored.add(tile);
                  });
      }

      return null;
  }
}

