pub struct RateLimiter {
    t_s: f32,
    limit: f32,
    y: f32,
}

impl RateLimiter {
    pub fn new(t_s: f32, limit: f32) -> Self {
        Self { t_s, limit, y: 0. }
    }

    pub fn rate_limit(&mut self, u: f32) -> f32 {
        let rate = (u - self.y) / self.t_s;

        self.y = if rate > self.limit {
            // Limit rising rate
            self.y + self.t_s * self.limit
        } else if rate < -self.limit {
            // Limit falling rate
            self.y - self.t_s * self.limit
        } else {
            u
        };

        self.y
    }
}
