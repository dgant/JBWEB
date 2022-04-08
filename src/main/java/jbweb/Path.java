package jbweb;

import bwapi.Pair;
import bwapi.Position;
import bwapi.TilePosition;
import jps.Graph;
import jps.JPS;
import jps.Tile;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import static jbweb.Pathfinding.maxCacheSize;
import static jbweb.Pathfinding.unitPathCache;

public class Path {
    private List<TilePosition> tiles;
    private double dist;
    private boolean reachable;
    private TilePosition source, target;

    public Path() {
        tiles = new ArrayList<>();
        dist = 0.0;
        reachable = false;
        source = TilePosition.Invalid;
        target = TilePosition.Invalid;
    }

    /// Returns the vector of TilePositions associated with this Path.
    public List<TilePosition> getTiles() {
        return tiles;
    }

    /// Returns the source (start) TilePosition of the Path.
    public TilePosition getSource() {
        return source;
    }

    /// Returns the target (end) TilePosition of the Path.
    public TilePosition getTarget() {
        return target;
    }

    /// Returns the distance from the source to the target in pixels.
    public double getDistance() {
        return dist;
    }

    /// Returns a check if the path was able to reach the target.
    public boolean isReachable() {
        return reachable;
    }

    private List<List<jps.Tile>> arrayToTileList(boolean[][] walkGrid) {
        List<List<Tile>> tiles = new ArrayList<>();
        for (int y = 0; y < walkGrid.length; y++) {
            List<Tile> tileRow = new ArrayList<>();
            for (int x = 0; x < walkGrid[y].length; x++) {
                Tile tile = new Tile(x, y);
                if (JBWEB.walkGrid[y][x]) {
                    tile.setWalkable(true);
                } else {
                    tile.setWalkable(false);
                }
                tileRow.add(tile);
            }
            tiles.add(tileRow);
        }
        return tiles;
    }

    /// Creates a path from the source to the target using JPS and collision provided by BWEB based on walkable tiles and used tiles.
    public void createUnitPath(Position s, Position t, Wall wall) {
        target = new TilePosition(t);
        source = new TilePosition(s);

        // If this path does not exist in cache, remove last reference and erase reference
        Pair<TilePosition, TilePosition> pathPoints = new Pair<>(source, target);
        if (unitPathCache.indexList.get(pathPoints) == null) {
            if (unitPathCache.pathCache.size() == maxCacheSize) {
                Path last = unitPathCache.pathCache.get(unitPathCache.pathCache.size()-1);
                unitPathCache.pathCache.remove(last);
                unitPathCache.indexList.remove(new Pair<>(last.getSource(), last.getTarget()));
            }
        }

        // If it does exist, set this path as cached version, update reference and push cached path to the front
        else {
            Path oldPath = unitPathCache.indexList.get(pathPoints).get(unitPathCache.pathCacheIndex);
            dist = oldPath.getDistance();
            tiles = oldPath.getTiles();
            reachable = oldPath.isReachable();

            unitPathCache.pathCache.remove(unitPathCache.indexList.get(pathPoints).get(unitPathCache.pathCacheIndex));
            List<Path> tmpCache = new ArrayList<>();
            tmpCache.add(this);
            tmpCache.addAll(unitPathCache.pathCache);
            unitPathCache.pathCache = tmpCache;
            unitPathCache.pathCacheIndex = 0;
            return;
        }

        // If not reachable based on previous paths to this area
        if (target.isValid(JBWEB.game) && JBWEB.mapBWEM.getMap().getArea(target) != null && wall.wallWalkable(new TilePosition(source.x, source.y))) {
            Integer checkReachable = unitPathCache.notReachableThisFrame.get(JBWEB.mapBWEM.getMap().getArea(target));
            if (checkReachable != null && checkReachable >= JBWEB.game.getFrameCount() && JBWEB.game.getFrameCount() > 0) {
                reachable = false;
                dist = Double.MAX_VALUE;
                return;
            }
        }

        // If we found a path, store what was found
        List<List<Tile>> grid = arrayToTileList(JBWEB.walkGrid);
        JPS<jps.Tile> jps = JPS.JPSFactory.getJPS(new Graph<>(grid), Graph.Diagonal.NO_OBSTACLES);
        Queue<jps.Tile> path = jps.findPathSync(new jps.Tile(source.x, source.y), new jps.Tile(target.x, target.y));
        if (path != null) {
            Position current = s;
            for (jps.Tile jpsTile : path) {
                TilePosition tile = new TilePosition(jpsTile.getX(), jpsTile.getY());
                dist += new Position(tile).getDistance(current);
                current = new Position(tile);
                tiles.add(tile);
            }
            reachable = true;

            // Update cache
            List<Path> tmpCache = new ArrayList<>();
            tmpCache.add(this);
            tmpCache.addAll(unitPathCache.pathCache);
            unitPathCache.pathCache = tmpCache;
            unitPathCache.pathCacheIndex = 0;
        }

        // If not found, set destination area as unreachable for this frame
        else if (target.isValid(JBWEB.game) && JBWEB.mapBWEM.getMap().getArea(target) != null) {
            dist = Double.MAX_VALUE;
            unitPathCache.notReachableThisFrame.put(JBWEB.mapBWEM.getMap().getArea(target), JBWEB.game.getFrameCount());
            reachable = false;
        }
    }

    /// Creates a path from the source to the target using BFS.
    public void bfsPath(Position bfsSourceP, Position bfsTargetP, Wall wall) {
        TilePosition bfsSource = new TilePosition(bfsSourceP);
        TilePosition bfsTarget = new TilePosition(bfsTargetP);
        List<TilePosition> direction = new ArrayList<>();
        direction.add(new TilePosition(0, 1));
        direction.add(new TilePosition(1, 0));
        direction.add(new TilePosition(-1, 0));
        direction.add(new TilePosition(0, -1));

        if (bfsSource.equals(bfsTarget)
                || bfsSource.equals(new TilePosition(0, 0))
                || bfsTarget.equals(new TilePosition(0, 0)))
            return;

        TilePosition[][] parentGrid = new TilePosition[256][256];

        Queue<TilePosition> nodeQueue = new LinkedList<>();
        nodeQueue.add(bfsSource);
        parentGrid[bfsSource.x][bfsSource.y] = bfsSource;

        // While not empty, pop off top the closest TilePosition to target
        while (!nodeQueue.isEmpty()) {
            TilePosition tile = nodeQueue.peek();
            nodeQueue.remove();

            for (TilePosition d : direction) {
                TilePosition next = new TilePosition(tile.x + d.x, tile.y + d.y);

                if (next.isValid(JBWEB.game)) {
                    // If next has a parent or is a collision, continue
                    TilePosition existingParent = parentGrid[next.x][next.y];
                    if (existingParent != null || !wall.wallWalkable(next))
                        continue;

                    // Check diagonal collisions where necessary
                    if ((d.x == 1 || d.x == -1) && (d.y == 1 || d.y == -1) && (!wall.wallWalkable(new TilePosition(tile.x + d.x, tile.y))
                            || !wall.wallWalkable(new TilePosition(tile.x, tile.y + d.y))))
                        continue;

                    // Set parent here
                    parentGrid[next.x][next.y] = tile;

                    // If at target, return path
                    if (next.equals(bfsTarget)) {
                        bfsPath_createPath(bfsSource, bfsTarget, parentGrid);
                        return;
                    }

                    nodeQueue.add(next);
                }
            }
        }
        reachable = false;
        dist = Double.MAX_VALUE;
    }

    // This function requires that parentGrid has been filled in for a path from source to target
    private void bfsPath_createPath(
            TilePosition bfsSource,
            TilePosition bfsTarget,
            TilePosition[][] parentGrid) {
        tiles.add(bfsTarget);
        reachable = true;
        TilePosition check = parentGrid[bfsTarget.x][bfsTarget.y];
        dist += new Position(bfsTarget).getDistance(new Position(check));

        do {
            tiles.add(check);
            TilePosition prev = check;
            check = parentGrid[check.x][check.y];
            dist += new Position(prev).getDistance(new Position(check));
        } while (check != bfsSource);

        // HACK: Try to make it more accurate to positions instead of tiles
        Position correctionSource = new Position(tiles.get(tiles.size()-2)); // Second to last tile
        Position correctionTarget = new Position(tiles.get(1)); // Second tile
        dist += bfsSource.getDistance(correctionSource.toTilePosition());
        dist += bfsTarget.getDistance(correctionTarget.toTilePosition());
        dist -= 64.0;
    }
}
