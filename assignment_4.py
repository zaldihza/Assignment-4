import heapq
import time
import random

class TerrainMap:
    def __init__(self, width, height):
        """
        Inisialisasi peta terrain untuk navigasi drone
        
        Args:
            width: Lebar peta
            height: Tinggi peta
        """
        self.width = width
        self.height = height
        self.grid = [[1 for _ in range(width)] for _ in range(height)]  # Default elevation is 1
        self.start = None
        self.goal = None
        
    def set_elevation(self, x, y, elevation):
        """Set tinggi elevasi pada posisi tertentu"""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = elevation
            return True
        return False
    
    def add_no_fly_zone(self, x, y):
        """Tambahkan zona larangan terbang pada posisi tertentu"""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = '#'  # Zona larangan terbang ditandai dengan '#'
            return True
        return False
    
    def set_start(self, x, y):
        """Set posisi awal drone"""
        if 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] != '#':
            self.start = (x, y)
            return True
        return False
    
    def set_goal(self, x, y):
        """Set posisi tujuan drone"""
        if 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] != '#':
            self.goal = (x, y)
            return True
        return False
    
    def is_valid_position(self, x, y):
        """Periksa apakah posisi valid dan bukan zona larangan terbang"""
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                self.grid[y][x] != '#')
    
    def get_elevation_cost(self, current, neighbor):
        """
        Hitung biaya perubahan elevasi antara dua posisi
        
        Args:
            current: Tuple (x, y) posisi saat ini
            neighbor: Tuple (x, y) posisi tetangga
        
        Returns:
            cost: Biaya pergerakan berdasarkan perubahan elevasi
        """
        x1, y1 = current
        x2, y2 = neighbor
        
        # Jika salah satu adalah zona larangan terbang, biaya tak terhingga
        if self.grid[y1][x1] == '#' or self.grid[y2][x2] == '#':
            return float('inf')
        
        # Hitung perubahan elevasi
        elevation_diff = abs(int(self.grid[y2][x2]) - int(self.grid[y1][x1]))
        
        # Biaya dasar + biaya tambahan untuk perubahan elevasi
        # Kenaikan elevasi lebih mahal dari penurunan
        if int(self.grid[y2][x2]) > int(self.grid[y1][x1]):
            # Naik - lebih berat
            return 1 + elevation_diff * 1.5
        else:
            # Turun - lebih ringan
            return 1 + elevation_diff * 0.5
    
    def generate_random_terrain(self, max_elevation=9, no_fly_zones=10):
        """
        Generate peta terrain acak dengan elevasi dan zona larangan terbang
        
        Args:
            max_elevation: Elevasi maksimum (1-9)
            no_fly_zones: Jumlah zona larangan terbang
        """
        # Reset grid
        self.grid = [[random.randint(1, max_elevation) for _ in range(self.width)] 
                     for _ in range(self.height)]
        
        # Tambahkan zona larangan terbang
        for _ in range(no_fly_zones):
            x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
            self.add_no_fly_zone(x, y)
        
        # Set posisi awal dan tujuan
        while True:
            x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
            if self.grid[y][x] != '#':
                self.set_start(x, y)
                break
        
        while True:
            x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
            if self.grid[y][x] != '#' and (x, y) != self.start:
                self.set_goal(x, y)
                break
    
    def print_map(self, path=None):
        """
        Cetak peta terrain dengan jalur terbang jika disediakan
        
        Args:
            path: List tuple (x, y) yang mewakili jalur terbang drone
        """
        # Buat salinan grid untuk visualisasi
        visual_grid = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                if (x, y) == self.start:
                    row.append('S')
                elif (x, y) == self.goal:
                    row.append('G')
                else:
                    row.append(str(self.grid[y][x]))
            visual_grid.append(row)
        
        # Tandai jalur dengan '*'
        if path:
            for x, y in path:
                if (x, y) != self.start and (x, y) != self.goal:
                    visual_grid[y][x] = '*'
        
        # Cetak peta
        for row in visual_grid:
            print(' '.join(row))

def a_star_search(terrain_map):
    """
    Implementasi algoritma A* untuk navigasi drone
    
    Args:
        terrain_map: Objek TerrainMap dengan informasi terrain
    
    Returns:
        path: Jalur terbang yang ditemukan
        visited_count: Jumlah node yang dikunjungi
        time_taken: Waktu eksekusi dalam milidetik
    """
    # Periksa apakah posisi awal dan tujuan sudah diatur
    if terrain_map.start is None or terrain_map.goal is None:
        return None, 0, 0
    
    # Mencatat waktu mulai
    start_time = time.time()
    
    # Ekstrak informasi posisi
    start = terrain_map.start
    goal = terrain_map.goal
    
    # Fungsi heuristik - menggunakan jarak Manhattan
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x2 - x1) + abs(y2 - y1)
    
    # Gerakan yang mungkin: atas, kanan, bawah, kiri
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    
    # Menyiapkan struktur data
    open_set = []  # Priority queue
    closed_set = set()
    g_score = {start: 0}  # Biaya sebenarnya dari start ke node
    f_score = {start: heuristic(start)}  # Estimasi total biaya
    parent = {}  # Untuk rekonstruksi jalur
    visited_count = 0
    
    # Format: (f_score, position)
    heapq.heappush(open_set, (f_score[start], start))
    
    while open_set:
        # Mengambil node dengan f_score terkecil
        _, current = heapq.heappop(open_set)
        visited_count += 1
        
        # Jika tujuan sudah ditemukan, rekonstruksi jalur dan kembalikan hasilnya
        if current == goal:
            # Rekonstruksi jalur
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            
            time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
            return path, visited_count, time_taken
        
        # Tandai node saat ini sebagai sudah dikunjungi
        closed_set.add(current)
        
        # Jelajahi semua tetangga yang mungkin
        x, y = current
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)
            
            # Periksa apakah posisi valid
            if not terrain_map.is_valid_position(nx, ny):
                continue
            
            # Jika sudah dikunjungi, lewati
            if neighbor in closed_set:
                continue
            
            # Hitung g_score baru
            tentative_g_score = g_score[current] + terrain_map.get_elevation_cost(current, neighbor)
            
            # Jika kita menemukan jalur yang lebih baik ke neighbor
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Simpan jalur yang lebih baik
                parent[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                
                # Tambahkan ke open set
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # Jika tidak ada jalur yang ditemukan
    time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
    return None, visited_count, time_taken

def greedy_best_first_search(terrain_map):
    """
    Implementasi algoritma Greedy Best-First Search untuk navigasi drone
    
    Args:
        terrain_map: Objek TerrainMap dengan informasi terrain
    
    Returns:
        path: Jalur terbang yang ditemukan
        visited_count: Jumlah node yang dikunjungi
        time_taken: Waktu eksekusi dalam milidetik
    """
    # Periksa apakah posisi awal dan tujuan sudah diatur
    if terrain_map.start is None or terrain_map.goal is None:
        return None, 0, 0
    
    # Mencatat waktu mulai
    start_time = time.time()
    
    # Ekstrak informasi posisi
    start = terrain_map.start
    goal = terrain_map.goal
    
    # Fungsi heuristik - menggunakan jarak Manhattan
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x2 - x1) + abs(y2 - y1)
    
    # Gerakan yang mungkin: atas, kanan, bawah, kiri
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    
    # Menyiapkan struktur data
    open_set = []  # Priority queue
    closed_set = set()
    parent = {}  # Untuk rekonstruksi jalur
    visited_count = 0
    
    # Format: (heuristic_value, position)
    heapq.heappush(open_set, (heuristic(start), start))
    
    while open_set:
        # Mengambil node dengan heuristic value terkecil
        _, current = heapq.heappop(open_set)
        visited_count += 1
        
        # Jika tujuan sudah ditemukan, rekonstruksi jalur dan kembalikan hasilnya
        if current == goal:
            # Rekonstruksi jalur
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            
            time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
            return path, visited_count, time_taken
        
        # Tandai node saat ini sebagai sudah dikunjungi
        closed_set.add(current)
        
        # Jelajahi semua tetangga yang mungkin
        x, y = current
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)
            
            # Periksa apakah posisi valid
            if not terrain_map.is_valid_position(nx, ny):
                continue
            
            # Jika sudah dikunjungi, lewati
            if neighbor in closed_set:
                continue
            
            # Jika belum ada parent untuk neighbor atau ini jalur baru
            if neighbor not in parent:
                parent[neighbor] = current
                # Tambahkan ke open set
                heapq.heappush(open_set, (heuristic(neighbor), neighbor))
    
    # Jika tidak ada jalur yang ditemukan
    time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
    return None, visited_count, time_taken

# Contoh penggunaan
if __name__ == "__main__":
    # Buat peta terrain berukuran 10x10
    terrain_map = TerrainMap(10, 10)
    
    # Generate terrain acak
    terrain_map.generate_random_terrain()
    
    # Cetak peta awal
    print("Peta Terrain:")
    terrain_map.print_map()
    
    # Jalankan algoritma A*
    print("\nMencari jalur terbang dengan A*...")
    a_star_path, a_star_visited, a_star_time = a_star_search(terrain_map)
    
    if a_star_path:
        print(f"A* berhasil menemukan jalur dengan {len(a_star_path)} langkah.")
        print(f"Jumlah node yang dikunjungi: {a_star_visited}")
        print(f"Waktu eksekusi: {a_star_time:.2f} ms")
        
        print("\nPeta dengan jalur A*:")
        terrain_map.print_map(a_star_path)
    else:
        print("A* tidak menemukan jalur.")
    
    # Jalankan algoritma Greedy Best-First Search
    print("\nMencari jalur terbang dengan Greedy Best-First Search...")
    gbfs_path, gbfs_visited, gbfs_time = greedy_best_first_search(terrain_map)
    
    if gbfs_path:
        print(f"GBFS berhasil menemukan jalur dengan {len(gbfs_path)} langkah.")
        print(f"Jumlah node yang dikunjungi: {gbfs_visited}")
        print(f"Waktu eksekusi: {gbfs_time:.2f} ms")
        
        print("\nPeta dengan jalur GBFS:")
        terrain_map.print_map(gbfs_path)
    else:
        print("GBFS tidak menemukan jalur.")
    
    # Perbandingan hasil
    print("\nPerbandingan A* vs GBFS:")
    if a_star_path and gbfs_path:
        print(f"Panjang jalur A*: {len(a_star_path)} langkah")
        print(f"Panjang jalur GBFS: {len(gbfs_path)} langkah")
        print(f"Node yang dikunjungi A*: {a_star_visited}")
        print(f"Node yang dikunjungi GBFS: {gbfs_visited}")
        print(f"Waktu eksekusi A*: {a_star_time:.2f} ms")
        print(f"Waktu eksekusi GBFS: {gbfs_time:.2f} ms")