curl -X POST http://127.0.0.1:5004/update_force -H "Content-Type: application/json" -d '[
    {"players": ["Team_A_Player_0", "Team_A_Player_1", "Team_B_Player_0", "Team_B_Player_1"],
     "forces": [12, 13, 11, 12], 
     "score": "ttt", 
     "rope_mark": 1
    }
]'
