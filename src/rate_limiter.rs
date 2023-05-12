pub struct RateLimiter {
    limit: f32,
    y: f32,
}

impl RateLimiter {
    pub fn new(limit: f32) -> Self {
        Self { limit, y: 0. }
    }

    pub fn rate_limit(&mut self, t_s: f32, u: f32) -> f32 {
        let rate = (u - self.y) / t_s;

        self.y = if rate > self.limit {
            // Limit rising rate
            self.y + t_s * self.limit
        } else if rate < -self.limit {
            // Limit falling rate
            self.y - t_s * self.limit
        } else {
            u
        };

        self.y
    }
}
