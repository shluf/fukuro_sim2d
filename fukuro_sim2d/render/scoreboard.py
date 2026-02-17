"""
scoreboard.py — Floating scoreboard widget untuk field rendering.
"""

import pygame
import pygame.freetype


class Scoreboard:
    """Floating scoreboard di pojok kiri atas lapangan."""
    
    def __init__(self):
        pygame.freetype.init()
        self.font_score = pygame.freetype.SysFont(
            "liberation mono, consolas, courier new", 28, bold=True)
        self.font_label = pygame.freetype.SysFont(
            "liberation mono, consolas, courier new", 14, bold=True)
        
        self.team_score = 0
        self.enemy_score = 0
        
        # UI colors
        self.bg_color = (20, 24, 32, 220)  # Semi-transparent dark
        self.border_color = (0, 188, 212)  # Cyan accent
        self.team_color = (39, 174, 96)    # Green
        self.enemy_color = (231, 76, 60)   # Red
        self.text_color = (220, 224, 235)  # Light gray
        self.reset_btn_color = (231, 76, 60)  # Red button
        
        self.reset_button_rect = pygame.Rect(0, 0, 0, 0)
    
    def reset_scores(self):
        """Reset semua skor ke 0."""
        self.team_score = 0
        self.enemy_score = 0
    
    def increment_team(self):
        """Team mencetak gol."""
        self.team_score += 1
    
    def increment_enemy(self):
        """Enemy mencetak gol."""
        self.enemy_score += 1
    
    def draw(self, screen: pygame.Surface, mode_name: str, margin_x: int = 10, margin_y: int = 10):
        """Draw scoreboard di pojok kiri atas.
        
        Parameters
        ----------
        screen : pygame.Surface
        mode_name : str  ('REGIONAL' atau 'NASIONAL')
        margin_x : int   margin dari kiri (pixel)
        margin_y : int   margin dari atas (pixel)
        """
        # Tentukan ukuran box berdasarkan mode
        if mode_name == 'REGIONAL':
            # Hanya team score
            box_w = 150
            box_h = 60
        else:
            # Team + Enemy score
            box_w = 250
            box_h = 70
        
        # Position
        box_x = margin_x
        box_y = margin_y
        
        # Draw semi-transparent background
        bg_surf = pygame.Surface((box_w, box_h), pygame.SRCALPHA)

        pygame.draw.rect(
            bg_surf,
            self.bg_color,
            (0, 0, box_w, box_h),
            border_radius=8
        )

        screen.blit(bg_surf, (box_x, box_y))

        
        # Draw border
        pygame.draw.rect(screen, self.border_color, 
                         (box_x, box_y, box_w, box_h), 2, border_radius=8)
        
        if mode_name == 'REGIONAL':
            # Regional: hanya team score
            label_surf, label_rect = self.font_label.render("TEAM", self.text_color)
            score_surf, score_rect = self.font_score.render(str(self.team_score), self.team_color)
            
            # Center align
            screen.blit(label_surf, (box_x + (box_w - label_rect.width) // 2, box_y + 12))
            screen.blit(score_surf, (box_x + (box_w - score_rect.width) // 2, box_y + 30))
        
        else:
            # Nasional: team vs enemy
            # Team (kiri)
            team_label, team_label_rect = self.font_label.render("TEAM", self.text_color)
            team_score_surf, team_score_rect = self.font_score.render(
                str(self.team_score), self.team_color)
            
            team_x = box_x + 25
            screen.blit(team_label, (team_x, box_y + 12))
            screen.blit(team_score_surf, (team_x, box_y + 35))
            
            # Separator
            sep_x = box_x + box_w // 2
            sep_surf, sep_rect = self.font_score.render(":", self.text_color)
            screen.blit(sep_surf, (sep_x - sep_rect.width // 2, box_y + 35))
            
            # Enemy (kanan)
            enemy_label, enemy_label_rect = self.font_label.render("ENEMY", self.text_color)
            enemy_score_surf, enemy_score_rect = self.font_score.render(
                str(self.enemy_score), self.enemy_color)
            
            enemy_x = box_x + box_w - 70
            screen.blit(enemy_label, (enemy_x, box_y + 12))
            screen.blit(enemy_score_surf, (enemy_x, box_y + 35))
        
        # Reset button (pojok kanan bawah scoreboard)
        btn_w = 60
        btn_h = 20
        btn_x = box_x + box_w + 8
        btn_y = (box_y + box_h) / 2 - ((btn_h - 8) / 2)
        
        self.reset_button_rect = pygame.Rect(btn_x, btn_y, btn_w, btn_h)
        pygame.draw.rect(screen, self.reset_btn_color, self.reset_button_rect, border_radius=4)
        
        reset_text, reset_rect = self.font_label.render("RESET", self.text_color)
        screen.blit(reset_text, (
            btn_x + (btn_w - reset_rect.width) // 2,
            btn_y + (btn_h - reset_rect.height) // 2
        ))
    
    def check_reset_click(self, pos: tuple[int, int]) -> bool:
        """Cek apakah klik mengenai tombol reset.
        
        Returns
        -------
        bool : True jika reset button diklik
        """
        return self.reset_button_rect.collidepoint(pos)
