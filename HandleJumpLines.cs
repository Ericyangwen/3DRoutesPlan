
using MathNet.Numerics.LinearAlgebra.Factorization;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using Util;

namespace GXDY_Designer.Controls.ProcessDesignerControls
{
    class Grid
    {
        public double x;
        public double y;
        public double z;
        public double f;
        public double g;
        public double h;
        public Grid parent = null;
        static double map_min_x;
        static double map_min_y;
        static double map_min_z;
        static double map_max_x;
        static double map_max_y;
        static double map_max_z;
        static int map_boundry_x;
        static int map_boundry_y;
        static int map_boundry_z;
        static bool[][][] arry3D_Map = null;
        static double pen_width;
        static double x_step;
        static double y_step;
        static double z_step;
        //与坐标相对应的索引值
        public int x_index;
        public int y_index;
        public int z_index;
        static List<double[]> allpoly_line_points = new List<double[]>();
        public Grid(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            x_index = (int)Math.Ceiling((this.x - map_min_x) / pen_width);
            y_index = (int)Math.Ceiling((this.y - map_min_y) / pen_width);
            z_index = (int)Math.Ceiling((this.z - map_min_z) / pen_width);
            this.f = 0;
            this.g = 0;
            this.h = 0;
        }

        /// <summary>
        /// 邻近节点不在openList中，设置“父节点”、G、H、F,并放入openList
        /// </summary>
        /// <param name="parent"></param>
        /// <param name="end"></param>
        public void InitGrid(Grid parent, Grid end)
        {
            this.parent = parent;
            //this.h = Math.Sqrt(Math.Pow(this.x_index - end.x_index, 2) + Math.Pow(this.y_index - end.y_index, 2) + Math.Pow(this.z_index - end.z_index, 2));
            this.h = Math.Abs(this.x_index - end.x_index) + Math.Abs(this.y_index - end.y_index) + Math.Abs(this.z_index - end.z_index);
            //欧氏距离
            if (parent == null)
            {
                this.g = 0;
            }
            else
            {
                this.g = parent.g + Math.Sqrt(Math.Pow(this.x_index - parent.x_index, 2) + Math.Pow(this.y_index - parent.y_index, 2) + Math.Pow(this.z_index - parent.z_index, 2));
            //this.g = parent.g + Math.Abs(this.x_index - parent.x_index) + Math.Abs(this.y_index - parent.y_index) + Math.Abs(this.z_index - parent.z_index);
            }
            this.f = this.g * 0.99 + this.h;
        }


        /// <summary>
        /// 初始化地图
        /// </summary>
        /// <param name="poly_line_points"></param>
        /// <param name="pen_width"></param>
        public static void initMap(List<double[]> poly_line_points, double in_pen_width)
        {
            map_min_x = double.MaxValue;
            map_min_y = double.MaxValue;
            map_min_z = double.MaxValue;
            map_max_x = double.MinValue;
            map_max_y = double.MinValue;
            map_max_z = double.MinValue;
            pen_width = in_pen_width;

            allpoly_line_points.Clear();
            allpoly_line_points.AddRange(poly_line_points);

            foreach (var ele in poly_line_points)
            {
                if (ele[0] < map_min_x)
                    map_min_x = ele[0];
                if (ele[0] > map_max_x)
                    map_max_x = ele[0];
                if (ele[1] < map_min_y)
                    map_min_y = ele[1];
                if (ele[1] > map_max_y)
                    map_max_y = ele[1];
                if (ele[2] < map_min_z)
                    map_min_z = ele[2];
                if (ele[2] > map_max_z)
                    map_max_z = ele[2];
            }
            map_boundry_x = (int)Math.Ceiling((map_max_x - map_min_x) / in_pen_width);
            map_boundry_y = (int)Math.Ceiling((map_max_y - map_min_y) / in_pen_width);
            map_boundry_z = (int)Math.Ceiling((map_max_z - map_min_z) / in_pen_width);

            arry3D_Map = new bool[map_boundry_x + 1][][]; //初始值为false
            for (int x = 0; x <= map_boundry_x; x++)
            {
                arry3D_Map[x] = new bool[map_boundry_y+1][];
                for (int y = 0; y <= map_boundry_y; y++)
                {
                    arry3D_Map[x][y] = new bool[map_boundry_z + 1];
                    for (int z = 0; z <= map_boundry_z; z++)
                    {
                        arry3D_Map[x][y][z] = false;
                    }
                }
            }

            //可行的路径
            foreach (var ele in poly_line_points)
            {
                int x = (int)Math.Ceiling((ele[0] - map_min_x) / in_pen_width);
                int y = (int)Math.Ceiling((ele[1] - map_min_y) / in_pen_width);
                int z = (int)Math.Ceiling((ele[2] - map_min_z) / in_pen_width);
                arry3D_Map[x][y][z] = true;
            }

            x_step = (map_max_x - map_min_x) / map_boundry_x;
            y_step = (map_max_y - map_min_y) / map_boundry_y;
            z_step = (map_max_z - map_min_z) / map_boundry_z;
        }

        /// <summary>
        /// 判断当前节点是否存在于openList或者closeList中
        /// </summary>
        /// <param name="grids"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        private static bool ContainGrid(List<Grid> grids, double x, double y, double z)
        {
            int tmp_x = (int)Math.Ceiling((x - map_min_x) / pen_width);
            int tmp_y = (int)Math.Ceiling((y - map_min_y) / pen_width);
            int tmp_z = (int)Math.Ceiling((z - map_min_z) / pen_width);
            foreach (var item in grids)
            {
                if (Math.Abs(item.x_index - tmp_x) < 0.0001 && Math.Abs(item.y_index - tmp_y) < 0.0001 && Math.Abs(item.z_index - tmp_z) < 0.0001)
                    return true;
            }
            return false;
        }

        /// <summary>
        /// 判断当前节点是否符合条件
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="openList"></param>
        /// <param name="closeList"></param>
        /// <returns></returns>
        private static bool IsValidGrid(double x, double y, double z, List<Grid> openList, List<Grid> closeList)
        {
            int tmp_x = (int)Math.Ceiling((x - map_min_x) / pen_width);
            int tmp_y = (int)Math.Ceiling((y - map_min_y) / pen_width);
            int tmp_z = (int)Math.Ceiling((z - map_min_z) / pen_width);

            //是否超过边界
            if (tmp_x < 0 || tmp_x >= map_boundry_x + 1 || tmp_y < 0 || tmp_y >= map_boundry_y + 1 || tmp_z < 0 || tmp_z >= map_boundry_z + 1)
                return false;
            //是否有障碍物
            if (!arry3D_Map[tmp_x][tmp_y][tmp_z])
                return false;
            //是否已经在closeList中
            if (ContainGrid(closeList, x, y, z))
                return false;
            ////是否已经在openList中
            //if (ContainGrid(openList, x, y, z))
            //    return false;

            ////是否已经在openList中
            if (!ContainGrid(openList, x, y, z))
                return true;
            //是否已经在openList中
            if (ContainGrid(openList, x, y, z))
                return true;
            return true;
        }

        /// <summary>
        /// 从当前节点周围的26个节点中找到符合条件的节点
        /// </summary>
        /// <param name="grid"></param>
        /// <param name="openList"></param>
        /// <param name="closeList"></param>
        /// <returns></returns>
        private static List<Grid> FindNeighbors(Grid grid, List<Grid> openList, List<Grid> closeList)
        {
            List<Grid> gridList = new List<Grid>();
            //同一层
            if (IsValidGrid(grid.x, grid.y - y_step, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y - y_step, grid.z));
            if (IsValidGrid(grid.x, grid.y + y_step, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y + y_step, grid.z));
            if (IsValidGrid(grid.x - x_step, grid.y, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y, grid.z));
            if (IsValidGrid(grid.x + x_step, grid.y, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y, grid.z));
            if (IsValidGrid(grid.x - x_step, grid.y - y_step, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y - y_step, grid.z));
            if (IsValidGrid(grid.x - x_step, grid.y + y_step, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y + y_step, grid.z));
            if (IsValidGrid(grid.x + x_step, grid.y - y_step, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y - y_step, grid.z));
            if (IsValidGrid(grid.x + x_step, grid.y + y_step, grid.z, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y + y_step, grid.z));

            //上一层
            if (IsValidGrid(grid.x, grid.y - y_step, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y - y_step, grid.z + z_step));
            if (IsValidGrid(grid.x, grid.y + y_step, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y + y_step, grid.z + z_step));
            if (IsValidGrid(grid.x - x_step, grid.y, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y, grid.z + z_step));
            if (IsValidGrid(grid.x + x_step, grid.y, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y, grid.z + z_step));
            if (IsValidGrid(grid.x - x_step, grid.y - y_step, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y - y_step, grid.z + z_step));
            if (IsValidGrid(grid.x - x_step, grid.y + y_step, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y + y_step, grid.z + z_step));
            if (IsValidGrid(grid.x + x_step, grid.y - y_step, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y - y_step, grid.z + z_step));
            if (IsValidGrid(grid.x + x_step, grid.y + y_step, grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y + y_step, grid.z + z_step));
            if (IsValidGrid(grid.x, grid.y , grid.z + z_step, openList, closeList))
                gridList.Add(new Grid(grid.x , grid.y , grid.z + z_step));

            //下一层
            if (IsValidGrid(grid.x, grid.y - y_step, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y - y_step, grid.z - z_step));
            if (IsValidGrid(grid.x, grid.y + y_step, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y + y_step, grid.z - z_step));
            if (IsValidGrid(grid.x - x_step, grid.y, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y, grid.z - z_step));
            if (IsValidGrid(grid.x + x_step, grid.y, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y, grid.z - z_step));
            if (IsValidGrid(grid.x - x_step, grid.y - y_step, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y - y_step, grid.z - z_step));
            if (IsValidGrid(grid.x - x_step, grid.y + y_step, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x - x_step, grid.y + y_step, grid.z - z_step));
            if (IsValidGrid(grid.x + x_step, grid.y - y_step, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y - y_step, grid.z - z_step));
            if (IsValidGrid(grid.x + x_step, grid.y + y_step, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x + x_step, grid.y + y_step, grid.z - z_step));
            if (IsValidGrid(grid.x, grid.y, grid.z - z_step, openList, closeList))
                gridList.Add(new Grid(grid.x, grid.y, grid.z - z_step));

            return gridList;
        }

        /// <summary>
        /// 从openList中找到F值最小的节点
        /// </summary>
        /// <param name="openList"></param>
        /// <returns></returns>
        private static Grid FindMinGrid(List<Grid> openList)
        {
            Grid tempGrid = openList[0];
            foreach (var item in openList)
            {
                if (item.f < tempGrid.f)
                    tempGrid = item;
            }
            return tempGrid;
        }

        /// <summary>
        /// A*寻路主逻辑
        /// </summary>
        /// <param name="star">起点</param>
        /// <param name="end">终点</param>
        /// <returns></returns>
        public static Grid AStarSearch(Grid start, Grid end,ref List<double[]> in_point_line )
        {
            List<Grid> openList = new List<Grid>();
            List<Grid> closeList = new List<Grid>();
            List<Grid> replace_point_list = new List<Grid>();
            //把起点加入openList
            openList.Add(start);
            Grid end_point_in_path = null;
            //主循环，每一轮检查1个当前方格节点
            while (openList.Count > 0)
            {
                //在openList中查找F值最小的节点
                Grid currentGrid = FindMinGrid(openList);
                //将当前节点从openList中移除
                openList.Remove(currentGrid);
                if (openList.Count == 0)
                {
                    end_point_in_path = currentGrid;
                }
                //当前方格节点进入closeList
                closeList.Add(currentGrid);
                //找到所有邻近节点
                List<Grid> neighbors = FindNeighbors(currentGrid, openList, closeList);
                foreach (var item in neighbors)
                {
                    int i = 0;
                    for (; i < openList.Count; i++)
                    {
                        if (item.x_index == openList[i].x_index && item.y_index == openList[i].y_index && item.z_index == openList[i].z_index)
                        {
                            break;
                        }
                    }
                    if (i == openList.Count)
                    {
                        //邻近节点不在openList中，设置“父节点”、G、H、F,并放入openList
                        item.InitGrid(currentGrid, end);
                        openList.Add(item);
                    }
                    else
                    {
                        item.InitGrid(currentGrid.parent, end);
                        if (openList[i].f > item.f)   //此处换成f的比较也可以,但是g更有说明性，代表已经走过的距离（代价），即已经有的代价最小
                        {
                            //replace_point_list.Add(openList[i]);
                            
                            openList[i].g = item.g;
                            openList[i].h = item.h;
                            openList[i].f = item.f;
                            if (closeList.Contains(item))
                            {
                                closeList.Remove(item);
                            }
                        }

                    }
                }
                //如果终点在openList中，直接返回终点格子
                bool is_end_point = false;
                foreach (var item in openList)
                {
                    if (Math.Abs(item.x_index - end.x_index) < 0.0001 && Math.Abs(item.y_index - end.y_index) < 0.0001 && Math.Abs(item.z_index - end.z_index) < 0.0001)
                    {
                        end_point_in_path = item;
                        is_end_point = true;
                        break;
                        //return item;
                    }
                }
                if (is_end_point)
                    break;
            }

            List<Grid> path_point = new List<Grid>();
            List<int[]> path_index = new List<int[]>();
            while (end_point_in_path != null)
            {
                path_point.Add(new Grid(end_point_in_path.x, end_point_in_path.y, end_point_in_path.z));

                path_index.Add(new int[] { end_point_in_path.x_index, end_point_in_path.y_index, end_point_in_path.z_index });
                end_point_in_path = end_point_in_path.parent;
            }
            List<double[]> path_in_poly_line = new List<double[]>();

            foreach (var p in path_index)
            {
                foreach (var ele in allpoly_line_points)
                {
                    int x = (int)Math.Ceiling((ele[0] - map_min_x) / pen_width);
                    int y = (int)Math.Ceiling((ele[1] - map_min_y) / pen_width);
                    int z = (int)Math.Ceiling((ele[2] - map_min_z) / pen_width);
                    if (p[0] == x && p[1] == y && p[2] == z)
                    {
                        path_in_poly_line.Add(new double[] { ele[0], ele[1], ele[2] });
                        break;
                    }
                }
            }
            in_point_line.AddRange(path_in_poly_line);
            //DebugTool.writeLog(path_in_poly_line, "path12.txt", true);
            List<double[]> path_in_poly_line_1 = new List<double[]>();
            foreach (var ss in replace_point_list)
            {
                path_in_poly_line_1.Add(new double[] { ss.x, ss.y, ss.z });
            }
            //DebugTool.writeLog(path_in_poly_line_1, "path12_1.txt", true);
            return end_point_in_path;
        }
    }
    
}

